"""Exercise the V2 rule-constrained A* + LOS demo with headless Chromium.

Requires Python Playwright and its Chromium browser. Opens the HTML with file://,
performs real button/checkbox/select interactions, and saves a JSON report plus
desktop/mobile screenshots. Failures are recorded before returning a nonzero exit.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path
import traceback

from playwright.sync_api import sync_playwright


PARAMETER_IDS = ["own-speed", "target-speed", "target-course", "safety-distance", "lookahead"]
EXPECTED_DEMO_VERSION = "2.0"


@contextmanager
def screenshot_on_failure(get_page, output: Path, report: dict):
    """Capture the failed UI before Playwright's outer context closes."""
    try:
        yield
    except Exception:
        try:
            failure_path = output / "failure.png"
            get_page().screenshot(path=str(failure_path), full_page=True)
            report["screenshots"].append(str(failure_path))
        except Exception:
            pass
        raise


def state(page) -> dict:
    return page.evaluate("window.demoApp.getState()")


def time_of(snapshot: dict) -> float:
    value = snapshot.get("time", snapshot.get("simulation", {}).get("time"))
    assert isinstance(value, (int, float)), f"No numeric simulation time in getState(): {list(snapshot)}"
    return value


def ui_values(page) -> dict:
    return {name: page.locator(f"#{name}").input_value() for name in PARAMETER_IDS}


def canvas_hash(page) -> str:
    """Compare actual rendered pixels when changing a display-only layer."""
    pixels = page.locator("#chart").evaluate("canvas => canvas.toDataURL()")
    return hashlib.sha256(pixels.encode("ascii")).hexdigest()


def polygon_area(points: list) -> float:
    xy = [(p["x"], p["y"]) if isinstance(p, dict) else (p[0], p[1]) for p in points]
    assert len(xy) >= 3 and all(math.isfinite(x + y) for x, y in xy)
    return abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(xy, xy[1:] + xy[:1]))) / 2


def search_evidence(snapshot: dict) -> dict:
    """Require non-degenerate constraints and search output, not a label alone."""
    assert snapshot.get("meta", {}).get("version") == EXPECTED_DEMO_VERSION, "Expected the V2 engine, not a V1 HTML build"
    zones = snapshot["ruleZones"]
    assert zones and any(zone["kind"] == "rule" for zone in zones), "Head-on should have a real rule constraint polygon"
    areas = {}
    for zone in zones:
        assert zone["kind"] in ("rule", "safety") and zone["label"] and zone["reason"]
        areas[zone["id"]] = polygon_area(zone["points"])
        assert areas[zone["id"]] > 0, f"Degenerate zone: {zone['id']}"
    info = snapshot["searchInfo"]
    assert "A*" in info["algorithm"], info
    assert isinstance(info["expanded"], int) and info["expanded"] > 0
    assert len(info["rawRoute"]) > 2 and info["closed"], "No underlying graph-search evidence"
    assert info["searchCount"] > 0 and info["gridSize"] > 0
    assert info["activeConstraints"], "Search must record which constraints were active"
    cost = info["costBreakdown"]
    assert all(math.isfinite(cost[key]) and cost[key] >= 0 for key in ("length", "routeDeviation", "total"))
    assert cost["length"] > 0 and cost["total"] >= cost["length"]
    validation = snapshot["routeValidation"]
    for key in ("staticSafe", "ruleSafe", "dynamicSafe"):
        assert validation[key] is True, f"Default head-on should select a trajectory that passes {key}: {validation}"
    chosen = [candidate for candidate in snapshot["candidates"] if candidate.get("chosen")]
    assert len(chosen) == 1 and chosen[0]["safe"] is True, "Expected one executable, validated candidate"
    assert math.isfinite(chosen[0]["cost"]) and chosen[0]["cost"] >= 0, "Chosen candidate is missing a numeric selection cost"
    assert chosen[0]["speed"] > 0 and snapshot["speedLimit"] > 0, f"Default head-on must have an executable moving avoidance candidate, not only a safe braking candidate: {chosen[0]['label']}, speed={chosen[0]['speed']}, phase={snapshot['decision']['phase']}"
    assert chosen[0].get("search"), "Chosen head-on route must have real graph-search provenance"
    assert info["rawRoute"] == chosen[0]["search"]["rawRoute"], "Search panel raw route does not describe the selected candidate"
    assert info["costBreakdown"] == chosen[0]["search"]["costBreakdown"], "Search panel cost does not describe the selected graph path"
    return {"zone_areas_m2": areas, "expanded": info["expanded"], "raw_waypoints": len(info["rawRoute"]), "search_count": info["searchCount"], "active_constraints": info["activeConstraints"], "cost_breakdown": cost, "route_validation": validation, "chosen_label": chosen[0]["label"], "chosen_speed_mps": chosen[0]["speed"], "chosen_cost": chosen[0]["cost"], "speed_limit_mps": snapshot["speedLimit"], "initial_phase": snapshot["decision"]["phase"]}


def no_horizontal_overflow(page) -> dict:
    dimensions = page.evaluate("""() => ({
      viewport: window.innerWidth,
      document: document.documentElement.scrollWidth,
      body: document.body.scrollWidth,
      canvas: document.querySelector('#chart').getBoundingClientRect().toJSON()
    })""")
    assert max(dimensions["document"], dimensions["body"]) <= dimensions["viewport"] + 1, dimensions
    assert dimensions["canvas"]["width"] >= 100 and dimensions["canvas"]["height"] >= 100, dimensions
    return dimensions


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--html", type=Path, default=Path(__file__).with_name("colregs-demo.html"))
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).with_name("validation") / "v2")
    args = parser.parse_args()
    html = args.html.resolve()
    assert html.is_file(), f"HTML not generated yet: {html}"
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = {"validation_target": "colregs_demo_v2", "expected_demo_version": EXPECTED_DEMO_VERSION, "html": str(html), "html_sha256": hashlib.sha256(html.read_bytes()).hexdigest(), "passed": False, "checks": [], "page_errors": [], "console_errors": [], "external_requests": [], "screenshots": []}
    page = None

    def passed(name: str, **details) -> None:
        report["checks"].append({"name": name, "passed": True, **details})

    try:
        with sync_playwright() as playwright, screenshot_on_failure(lambda: page, output, report):
            browser = playwright.chromium.launch(headless=True)
            report["browser_version"] = browser.version
            context = browser.new_context(viewport={"width": 1365, "height": 900}, accept_downloads=True, reduced_motion="reduce")
            page = context.new_page()
            page.on("pageerror", lambda error: report["page_errors"].append(str(error)))
            page.on("console", lambda message: report["console_errors"].append(message.text) if message.type == "error" else None)
            page.on("request", lambda request: report["external_requests"].append(request.url) if request.url.startswith(("https://", "http://")) else None)
            page.goto(html.as_uri(), wait_until="networkidle")
            page.wait_for_function("window.demoApp && typeof window.demoApp.getState === 'function'")
            for selector in ["#scene-list", "#play-btn", "#step-btn", "#reset-btn", "#speed-select", "#rules-toggle", "#cooperative-toggle", "#chart", "#export-btn", "#show-rules", "#show-search"]:
                assert page.locator(selector).is_visible(), f"Missing control {selector}"
            for name in PARAMETER_IDS:
                assert page.locator(f"#{name}").is_visible()
            scene_ids = page.locator("#scene-list [data-scene]").evaluate_all("elements => elements.map(e => e.dataset.scene)")
            assert len(scene_ids) >= 2 and len(set(scene_ids)) == len(scene_ids), scene_ids
            initial = state(page)
            assert time_of(initial) == 0, initial
            assert initial["scenarioId"] == "head-on", "Default verification scenario must be head-on"
            initial_evidence = search_evidence(initial)
            original_global_route = initial["globalRoute"]
            assert initial["activeRoute"] != original_global_route, "Rule-constrained route must differ from the original head-on global route"
            report["demo_version"] = initial["meta"]["version"]
            passed("v2_real_rule_polygons_and_astar_evidence", **initial_evidence)
            for selector in ["#search-status", "#search-algorithm", "#search-trigger", "#search-count", "#search-expanded", "#search-version", "#search-grid", "#search-rejoin", "#active-constraints", "#check-static", "#check-rule", "#check-dynamic", "#validation-reason", "#candidate-list"]:
                assert page.locator(selector).is_visible(), f"Missing V2 evidence panel {selector}"
            assert page.locator("#search-algorithm").inner_text() == initial["searchInfo"]["algorithm"]
            assert page.locator("#search-count").inner_text() == str(initial["searchInfo"]["searchCount"])
            assert page.locator("#search-expanded").inner_text() == str(initial["searchInfo"]["expanded"])
            assert page.locator("#search-status").get_attribute("data-status") == initial["searchInfo"]["status"]
            for selector in ("#check-static", "#check-rule", "#check-dynamic"):
                assert page.locator(selector).get_attribute("data-result") == "pass"
            assert page.locator('#candidate-list [data-chosen="true"]').count() == 1
            passed("v2_search_and_validation_panels_match_engine")
            default_values = ui_values(page)
            desktop_dimensions = no_horizontal_overflow(page)
            painted = page.locator("#chart").evaluate("""canvas => {
              const ctx=canvas.getContext('2d'), data=ctx.getImageData(0,0,canvas.width,canvas.height).data;
              const colors=new Set(); for(let i=0;i<data.length;i+=4*31) colors.add(`${data[i]},${data[i+1]},${data[i+2]},${data[i+3]}`);
              return colors.size;
            }""")
            assert painted > 3, f"Canvas contains only {painted} sampled colors"
            desktop_path = output / "desktop-initial.png"
            page.screenshot(path=str(desktop_path), full_page=True)
            report["screenshots"].append(str(desktop_path))
            passed("file_url_load_and_draw", scene_ids=scene_ids, dimensions=desktop_dimensions, sampled_canvas_colors=painted)

            assert page.locator("#show-rules").is_checked()
            assert not page.locator("#show-search").is_checked()
            assert initial["layers"]["rules"] is True and initial["layers"]["search"] is False
            rules_canvas = canvas_hash(page)
            page.locator("#show-rules").uncheck()
            no_rules_canvas = canvas_hash(page)
            assert no_rules_canvas != rules_canvas, "Rule-layer checkbox did not change the rendered polygons"
            page.locator("#show-rules").check()
            assert canvas_hash(page) == rules_canvas, "Rule-layer rendering did not restore"
            page.locator("#show-search").check()
            assert state(page)["layers"]["search"] is True
            search_canvas = canvas_hash(page)
            assert search_canvas != rules_canvas, "Search-layer checkbox did not draw expansion evidence"
            assert state(page)["searchInfo"]["searchCount"] == initial["searchInfo"]["searchCount"], "Display-only layer unexpectedly reran A*"
            assert state(page)["ruleZones"] == initial["ruleZones"], "Display-only layer changed planning constraints"
            search_path = output / "desktop-rule-search-layers.png"
            page.screenshot(path=str(search_path), full_page=True)
            report["screenshots"].append(str(search_path))
            page.locator("#show-search").uncheck()
            passed("v2_rule_and_search_layers_render_without_replanning")

            search_counts = [initial["searchInfo"]["searchCount"]]
            for _ in range(10):
                page.evaluate("window.demoApp.advance(0.5)")
                current = state(page)
                search_counts.append(current["searchInfo"]["searchCount"])
                assert current["globalRoute"] == original_global_route, "Local search modified the saved global route"
            assert all(b >= a for a, b in zip(search_counts, search_counts[1:])), search_counts
            reused_intervals = sum(a == b for a, b in zip(search_counts, search_counts[1:]))
            assert reused_intervals > 0, f"A* ran at every stable 0.5 s interval: {search_counts}"
            passed("v2_stable_route_search_reuse", seconds=5, interval_seconds=0.5, search_counts=search_counts, reused_intervals=reused_intervals)
            page.locator("#reset-btn").click()

            page.locator("#step-btn").click()
            after_step = time_of(state(page))
            assert after_step > 0
            page.wait_for_timeout(250)
            assert time_of(state(page)) == after_step, "Single step continued running"
            page.locator("#play-btn").click()
            page.wait_for_function("previous => window.demoApp.getState().time > previous", arg=after_step, timeout=5000)
            page.locator("#play-btn").click()
            paused_time = time_of(state(page))
            assert paused_time > after_step
            page.wait_for_timeout(250)
            assert time_of(state(page)) == paused_time, "Pause did not stop simulation"
            passed("single_step_play_pause", single_step_time=after_step, paused_time=paused_time)

            speeds = page.locator("#speed-select option").evaluate_all("els => els.map(e => e.value)")
            assert len(speeds) >= 2
            page.locator("#speed-select").select_option(speeds[-1])
            assert page.locator("#speed-select").input_value() == speeds[-1]
            assert state(page)["playbackRate"] == float(speeds[-1])
            passed("playback_speed_selection", selected=speeds[-1])

            for scene_id in scene_ids[1:] + scene_ids[:1]:
                page.locator(f'#scene-list [data-scene="{scene_id}"]').click()
                snapshot = state(page)
                assert time_of(snapshot) == 0, f"{scene_id} did not reset time"
                assert snapshot.get("scenarioId", snapshot.get("scenario", {}).get("id")) == scene_id, f"Wrong scenario after selecting {scene_id}"
                page.locator("#step-btn").click()
                assert time_of(state(page)) > 0
            passed("all_scenarios_select_and_step", count=len(scene_ids))

            page.locator("#reset-btn").click()
            assert time_of(state(page)) == 0
            page.locator("#rules-toggle").uncheck()
            assert not page.locator("#rules-toggle").is_checked()
            page.evaluate("window.demoApp.advance(5)")
            rules_off = state(page)
            assert time_of(rules_off) >= 4.9
            options = rules_off.get("config", {})
            assert options.get("rulesEnabled") is False, "Rules checkbox did not update the simulation"
            active = rules_off.get("activeRoute")
            global_route = rules_off.get("globalRoute")
            assert isinstance(active, list) and isinstance(global_route, list) and active == global_route, "Rules-off should use the original A* route"
            assert rules_off["ruleZones"] == [], "Disabling rules must clear actual planning zones"
            assert global_route == original_global_route, "Toggling rules changed the original global route"
            page.locator("#rules-toggle").check()
            assert page.locator("#rules-toggle").is_checked()
            passed("rules_off_clears_zones_and_preserves_global_route", simulated_seconds=time_of(rules_off), global_waypoints=len(global_route))

            original_cooperative = page.locator("#cooperative-toggle").is_checked()
            page.locator("#cooperative-toggle").set_checked(not original_cooperative)
            assert page.locator("#cooperative-toggle").is_checked() != original_cooperative
            assert state(page)["config"]["targetCooperative"] is (not original_cooperative), "Cooperation toggle did not reach the simulation"
            page.locator("#cooperative-toggle").set_checked(original_cooperative)
            passed("target_cooperation_toggle")

            page.locator("#reset-btn").click()
            default_values = ui_values(page)
            changed = {}
            for name in PARAMETER_IDS:
                locator = page.locator(f"#{name}")
                new_value = locator.evaluate("""el => {
                  const current=Number(el.value), min=Number(el.min || 0), max=Number(el.max || current+2), step=Number(el.step || 1);
                  return String(current+step<=max ? current+step : Math.max(min,current-step));
                }""")
                assert new_value != default_values[name], f"Cannot change {name}"
                locator.evaluate("""(el,value) => { el.value=value; el.dispatchEvent(new Event('input',{bubbles:true})); el.dispatchEvent(new Event('change',{bubbles:true})); }""", new_value)
                assert locator.input_value() == new_value, name
                changed[name] = new_value
            changed_state = state(page)
            for name, key in [("own-speed", "ownSpeed"), ("target-speed", "targetSpeed"), ("safety-distance", "safetyDistance"), ("lookahead", "lookahead")]:
                assert changed_state["config"][key] == float(changed[name]), f"{name} did not reach the simulation"
            expected_yaw = math.radians(90 - float(changed["target-course"]))
            actual_yaw = changed_state["target"]["yaw"]
            assert abs(math.atan2(math.sin(actual_yaw - expected_yaw), math.cos(actual_yaw - expected_yaw))) < 1e-9
            page.evaluate("window.demoApp.advance(2)")
            page.locator("#reset-btn").click()
            assert time_of(state(page)) == 0
            assert ui_values(page) == changed, {"expected": changed, "actual": ui_values(page)}
            # Restart preserves the experiment parameters; choosing a different
            # scenario and returning restores that scenario's defined defaults.
            page.locator(f'#scene-list [data-scene="{scene_ids[1]}"]').click()
            page.locator(f'#scene-list [data-scene="{scene_ids[0]}"]').click()
            assert ui_values(page) == default_values, {"expected": default_values, "actual": ui_values(page)}
            passed("parameter_changes_restart_and_scenario_defaults", defaults=default_values, changed=changed)

            page.locator("#chart").scroll_into_view_if_needed()
            before_drag = state(page)
            ship_pixel = page.evaluate("window.demoApp.screenPoint(window.demoApp.getState().own)")
            box = page.locator("#chart").bounding_box()
            assert box is not None
            page.mouse.move(box["x"] + ship_pixel["x"], box["y"] + ship_pixel["y"])
            page.mouse.down()
            page.mouse.move(box["x"] + ship_pixel["x"] + 20, box["y"] + ship_pixel["y"] + 25, steps=5)
            page.mouse.up()
            after_drag = state(page)
            moved = math.hypot(after_drag["own"]["x"] - before_drag["own"]["x"], after_drag["own"]["y"] - before_drag["own"]["y"])
            assert moved > 1 and time_of(after_drag) == 0, f"Ship drag did not update initial position: {moved}"
            before_zoom = page.evaluate("window.demoApp.screenPoint(window.demoApp.getState().target)")
            page.mouse.move(box["x"] + box["width"] / 2, box["y"] + box["height"] / 2)
            page.mouse.wheel(0, -250)
            page.wait_for_timeout(100)
            after_zoom = page.evaluate("window.demoApp.screenPoint(window.demoApp.getState().target)")
            assert math.hypot(after_zoom["x"] - before_zoom["x"], after_zoom["y"] - before_zoom["y"]) > 1, "Mouse wheel did not zoom the chart"
            page.locator("#fit-btn").click()
            after_fit = page.evaluate("window.demoApp.screenPoint(window.demoApp.getState().target)")
            assert math.hypot(after_fit["x"] - before_zoom["x"], after_fit["y"] - before_zoom["y"]) < 1e-8, "Fit button did not restore chart scale"
            page.locator(f'#scene-list [data-scene="{scene_ids[0]}"]').click()
            passed("drag_initial_ship_position_zoom_and_fit", moved_m=moved)

            page.evaluate("window.demoApp.advance(15)")
            with page.expect_download() as download_event:
                page.locator("#export-btn").click()
            download = download_event.value
            downloaded_path = output / download.suggested_filename
            download.save_as(str(downloaded_path))
            assert downloaded_path.stat().st_size > 50, "Export was empty"
            exported = json.loads(downloaded_path.read_text(encoding="utf-8"))
            live = state(page)
            assert exported["format"] == "colregs-los-demo-v2"
            assert exported["meta"]["version"] == EXPECTED_DEMO_VERSION
            assert exported["time"] == live["time"] and exported["own"] == live["own"]
            assert exported["parameters"] == live["config"]
            assert exported["trajectory"] and exported["history"] and exported["logs"]
            assert exported["globalRoute"] == live["globalRoute"] and exported["activeRoute"] == live["activeRoute"]
            for evidence_key in ("ruleZones", "routeValidation"):
                assert exported[evidence_key] == live[evidence_key], f"Export omitted or changed {evidence_key}"
            assert len(exported["candidates"]) == len(live["candidates"])
            for exported_candidate, live_candidate in zip(exported["candidates"], live["candidates"], strict=True):
                for key, value in live_candidate.items():
                    if key != "search":
                        assert exported_candidate[key] == value, f"Export changed candidate.{key}"
                live_search = live_candidate.get("search")
                if live_search:
                    candidate_summary = exported_candidate["search"]
                    for key in ("algorithm", "status", "expanded", "rawRoute", "costBreakdown"):
                        if key in live_search:
                            assert candidate_summary[key] == live_search[key], f"Candidate search summary changed {key}"
                    assert "closed" not in candidate_summary and "route" not in candidate_summary, "Candidate duplicated unbounded search nodes"
                    assert candidate_summary["closedOmitted"] is True
                    assert candidate_summary["closedTotal"] == len(live_search.get("closed", []))
                else:
                    assert exported_candidate.get("search") == live_search
            exported_info = exported["searchInfo"]
            for key, value in live["searchInfo"].items():
                if key != "closed":
                    assert exported_info[key] == value, f"Export changed searchInfo.{key}"
            live_closed = live["searchInfo"]["closed"]
            export_limit = exported_info["closedExportLimit"]
            assert isinstance(export_limit, int) and 0 < export_limit <= 1500
            assert exported_info["closed"] == live_closed[:export_limit]
            assert exported_info["closedTotal"] == len(live_closed)
            assert exported_info["closedTruncated"] is (len(live_closed) > export_limit)
            assert exported["searchInfo"]["rawRoute"] and exported["searchInfo"]["expanded"] > 0
            passed("export_download_and_data_consistency", filename=download.suggested_filename, bytes=downloaded_path.stat().st_size, fields=list(exported))
            page.locator("#toast").wait_for(state="hidden", timeout=5000)
            run_path = output / "desktop-running.png"
            page.screenshot(path=str(run_path), full_page=True)
            report["screenshots"].append(str(run_path))

            page.set_viewport_size({"width": 390, "height": 844})
            page.wait_for_timeout(100)
            mobile_dimensions = no_horizontal_overflow(page)
            mobile_path = output / "mobile.png"
            page.screenshot(path=str(mobile_path), full_page=True)
            report["screenshots"].append(str(mobile_path))
            page.locator("#reset-btn").click()
            page.locator("#step-btn").click()
            assert time_of(state(page)) > 0
            passed("mobile_layout_and_step", dimensions=mobile_dimensions)
            assert not report["page_errors"], report["page_errors"]
            assert not report["console_errors"], report["console_errors"]
            assert not report["external_requests"], report["external_requests"]
            passed("no_javascript_errors_or_external_requests")
            report["passed"] = True
            browser.close()
    except Exception as error:
        report["failure"] = {"message": str(error), "traceback": traceback.format_exc()}
        raise
    finally:
        report_path = output / "browser_report.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"report": str(report_path), "passed": report["passed"], "checks": len(report["checks"])}, ensure_ascii=False))


if __name__ == "__main__":
    main()
