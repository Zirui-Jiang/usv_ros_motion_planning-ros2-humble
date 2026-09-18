# 本地调参台 HTTP 入口：只提供静态页和 JSON API，规划全部走 engine 里的仓库源码。
"""Serve the Hybrid A* workbench from the repository tree."""
from __future__ import annotations

import argparse
import json
import sys
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from engine import (
    cost_grid_experiment, describe_world, heading_experiment, heuristic_experiment,
    inflate, occupancy_from_payload, plan, presets, sweep,
)

HERE = Path(__file__).resolve().parent
MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".md": "text/markdown; charset=utf-8",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}


def _json(handler, code, payload):
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(body)


def _read_json(handler):
    length = int(handler.headers.get("Content-Length") or 0)
    if length > 8_000_000:
        raise ValueError("request body is too large")
    raw = handler.rfile.read(length) if length else b"{}"
    if not raw:
        return {}
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid JSON: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("JSON body must be an object")
    return payload


# 只接受 JSON API 与本目录静态文件，规划计算全部转给 engine。
class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), format % args))

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            return self._file("page.html")
        if path == "/api/world":
            return _json(self, 200, describe_world())
        name = path.lstrip("/").replace("\\", "/")
        if not name or ".." in name.split("/"):
            self.send_error(404)
            return
        return self._file(name)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            body = _read_json(self)
            if path == "/api/inflate":
                params = body.get("params") or {}
                occupancy = occupancy_from_payload(body["occupancy"])
                costs = inflate(
                    occupancy,
                    inscribed_radius=float(params.get("inscribed_radius", 1.3)),
                    inflation_radius=float(params.get("inflation_radius", 3.0)),
                    cost_scaling_factor=float(params.get("cost_scaling_factor", 2.0)),
                )
                return _json(self, 200, {"costs": costs.astype(int).reshape(-1).tolist()})
            if path == "/api/plan":
                return _json(self, 200, plan(body["occupancy"], body["start"], body["goal"], body.get("params")))
            if path == "/api/sweep":
                return _json(self, 200, sweep(body["occupancy"], body["start"], body["goal"],
                                              body.get("params"), body["param"], body["values"]))
            if path == "/api/experiment/heading":
                return _json(self, 200, heading_experiment(
                    body["occupancy"], body["start"], body["goal"], body.get("params"),
                    body.get("samples", 12)))
            if path == "/api/experiment/heuristic":
                return _json(self, 200, heuristic_experiment(
                    body["occupancy"], body["start"], body["goal"], body.get("params")))
            if path == "/api/experiment/cost-grid":
                return _json(self, 200, cost_grid_experiment(
                    body["occupancy"], body["start"], body["goal"], body.get("params"),
                    body.get("factors", (0.05, 0.25, 0.5, 1.0)),
                    body.get("penalties", (0.0, 0.5, 1.0, 2.0))))
            self.send_error(404)
        except KeyError as error:
            _json(self, 400, {"error": f"missing field {error}"})
        except ValueError as error:
            _json(self, 400, {"error": str(error)})
        except Exception as error:
            traceback.print_exc()
            _json(self, 500, {"error": str(error)})

    def _file(self, name):
        target = (HERE / name).resolve()
        try:
            target.relative_to(HERE)
        except ValueError:
            self.send_error(404)
            return
        if not target.is_file():
            self.send_error(404)
            return
        data = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", MIME.get(target.suffix, "application/octet-stream"))
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


# 预热规划器导入，避免浏览器打开后第一次点击还在等 sys.path。
presets()


def main():
    parser = argparse.ArgumentParser(description="Hybrid A* 参数观察台（离线，调用仓库规划器）")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://{args.host}:{args.port}/"
    print(f"Hybrid A* tuner: {url}", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
        httpd.server_close()


if __name__ == "__main__":
    main()
