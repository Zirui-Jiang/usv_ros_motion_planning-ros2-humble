"""Exercise the rendered local UI; requires optional Playwright + Chromium."""
import argparse
import hashlib
import json
from pathlib import Path

from playwright.sync_api import sync_playwright, expect


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8766')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parents[3] / 'src/core/shared/usv_navigation/config/robots/otter/adrc_controller_params_otter.yaml'
    original = hashlib.sha256(source.read_bytes()).hexdigest()
    observations = {}
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={'width': 1440, 'height': 1050})
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto(args.url, wait_until='networkidle')
        expect(page.locator('#statusText')).to_contain_text('计算完成')
        expect(page.locator('#metricT90')).to_contain_text('67.1')
        observations['baseline_t90'] = page.locator('#metricT90').inner_text()

        # Rapid edits must leave charts/exports bound to the latest pair.
        page.locator('#kp').fill('40')
        page.locator('#ki').fill('4')
        expect(page.locator('#comparisonNote')).to_contain_text('当前 Kp 40 / Ki 4')
        expect(page.locator('#export')).to_be_enabled()
        observations['kp40_ki4_t90'] = page.locator('#metricT90').inner_text()
        assert float(observations['kp40_ki4_t90'].removesuffix('s')) < 67.1
        page.locator('#pin').click()
        page.locator('#kp').fill('60')
        page.locator('#ki').fill('8')
        expect(page.locator('#comparisonNote')).to_contain_text('当前 Kp 60 / Ki 8')
        expect(page.locator('#export')).to_be_enabled()
        expect(page.locator('#pinLegend')).to_contain_text('40')
        page.locator('#components').check()
        page.locator('#timeWindow').select_option('20')
        page.locator('#cursor').focus()
        page.keyboard.press('ArrowRight')
        expect(page.locator('#readout')).to_contain_text('0.1 s')

        # Screen a grid, including arbitrary decimal gain increments.
        page.locator('#kiMin').fill('0.7')
        page.locator('#kiMax').fill('10')
        page.locator('#sweep').click()
        expect(page.locator('#sweepStatus')).to_contain_text('已计算 25 组', timeout=60000)
        expect(page.locator('#candidates tr')).to_have_count(8)
        observations['sweep'] = page.locator('#sweepStatus').inner_text()
        page.locator('[data-candidate="0"]').click()
        expect(page.locator('#export')).to_be_enabled()
        observations['selected'] = {'kp': page.locator('#kp').input_value(), 'ki': page.locator('#ki').input_value()}
        page.locator('#timeWindow').select_option('full')
        page.screenshot(path=str(args.output / 'desktop.png'), full_page=True)
        page.locator('#export').click()
        expect(page.locator('#exportDialog')).to_be_visible()
        exported = Path(page.locator('#exportPath').inner_text())
        assert exported.is_file()
        assert '/mnt/' in page.locator('#launchCommand').input_value()
        observations['export_path'] = str(exported)
        page.locator('#exportDialog .close-dialog').click()

        # Changing scenarios clears incompatible pinned comparisons/search.
        page.locator('label:has(input[value="coast"])').click()
        expect(page.locator('#scenarioHint')).to_contain_text('切断推力')
        expect(page.locator('#export')).to_be_enabled()
        expect(page.locator('#pinLegend')).to_be_hidden()
        expect(page.locator('#candidateTable')).to_be_hidden()
        page.locator('#cursor').fill('800')
        page.locator('#cursor').dispatch_event('input')
        expect(page.locator('#readout')).to_contain_text('推力 0.00 N')
        observations['coast'] = page.locator('#readout').inner_text()
        page.locator('label:has(input[value="down"])').click()
        expect(page.locator('#scenarioHint')).to_contain_text('30%')
        expect(page.locator('#export')).to_be_enabled()
        observations['down_t90'] = page.locator('#metricT90').inner_text()

        page.locator('#kp').fill('-1')
        expect(page.locator('#error')).to_be_visible()
        expect(page.locator('#export')).to_be_disabled()
        page.locator('#reset').click()
        expect(page.locator('#export')).to_be_enabled()
        page.locator('#helpOpen').click()
        expect(page.locator('#helpDialog')).to_be_visible()
        page.keyboard.press('Escape')
        expect(page.locator('#helpDialog')).to_be_hidden()
        # Verify overflow at documented breakpoints and basic focus visibility.
        for width, height in [(375, 812), (768, 1024), (1024, 900)]:
            page.set_viewport_size({'width': width, 'height': height})
            assert page.evaluate('() => document.documentElement.scrollWidth <= innerWidth'), width
        page.set_viewport_size({'width': 375, 'height': 812})
        page.screenshot(path=str(args.output / 'mobile.png'), full_page=True)
        page.locator('#kp').focus()
        assert page.locator('#kp').evaluate('(el) => getComputedStyle(el).outlineStyle') != 'none'
        assert not errors, errors
        observations['browser_errors'] = errors
        browser.close()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original
    observations['source_yaml_unchanged'] = True
    observations['passed'] = True
    (args.output / 'browser-results.json').write_text(json.dumps(observations, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(observations, ensure_ascii=True))


if __name__ == '__main__':
    main()
