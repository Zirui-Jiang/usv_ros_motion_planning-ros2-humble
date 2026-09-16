#!/usr/bin/env python3
"""Start the offline Otter PI tuning interface; no ROS/Gazebo required."""
import argparse
import json
from pathlib import Path
import threading
import urllib.error
import urllib.request
import webbrowser

from speed_tuner.engine import CONFIG, DYNAMICS, Context
from speed_tuner.web import create_app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--controller-config', type=Path, default=CONFIG)
    parser.add_argument('--dynamics-config', type=Path, default=DYNAMICS)
    parser.add_argument('--export-dir', type=Path)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('port must be between 1024 and 65535')
    context = Context(args.controller_config, args.dynamics_config)
    url = f'http://127.0.0.1:{args.port}'
    # A double-click should reopen this same tool, not spawn a duplicate server.
    if args.export_dir is None:
        try:
            with urllib.request.urlopen(url + '/api/config', timeout=1) as stream:
                existing = json.loads(stream.read(100000))
            if (existing.get('source_id') == context.source_id
                    and existing.get('implementation_sha256') == context.metadata()['implementation_sha256']
                    and existing.get('config_path') == str(context.controller_path)):
                print(f'Existing Otter Speed Lab: {url}', flush=True)
                if not args.no_browser:
                    webbrowser.open(url)
                return
        except (urllib.error.URLError, OSError, ValueError):
            pass
    app = create_app(args.controller_config, args.dynamics_config, args.export_dir)
    print(f'Otter Speed Lab: {url}', flush=True)
    print('Local numerical model only. Close with Ctrl+C.', flush=True)
    if not args.no_browser:
        threading.Timer(1., lambda: webbrowser.open(url)).start()
    app.run(host='127.0.0.1', port=args.port, debug=False, use_reloader=False, threaded=True)


if __name__ == '__main__':
    main()
