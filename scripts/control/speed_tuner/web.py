"""Loopback-only UI server. Candidate exports never overwrite source YAML."""
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory
from werkzeug.exceptions import HTTPException

from .engine import CONFIG, DYNAMICS, Context, simulate, sweep, export_candidate


def create_app(controller_path=CONFIG, dynamics_path=DYNAMICS, export_dir=None):
    here = Path(__file__).resolve().parent
    app = Flask(__name__, static_folder=str(here / 'static'))
    app.config['MAX_CONTENT_LENGTH'] = 8192
    app.json.ensure_ascii = False
    output = Path(export_dir) if export_dir is not None else here / 'exports'

    @app.before_request
    def local_requests_only():
        if request.host.split(':')[0] not in ('localhost', '127.0.0.1'):
            return jsonify(error='仅允许本机访问'), 403
        if request.method == 'POST':
            origin = request.headers.get('Origin')
            if origin and origin != request.host_url.rstrip('/'):
                return jsonify(error='不接受跨来源请求'), 403
            if not request.is_json:
                return jsonify(error='请求必须是 JSON'), 415

    @app.after_request
    def headers(response):
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Content-Security-Policy'] = "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; frame-ancestors 'none'"
        return response

    @app.errorhandler(ValueError)
    def invalid(error):
        return jsonify(error=str(error)), 400

    @app.errorhandler(OSError)
    def file_error(error):
        return jsonify(error='文件读写失败：' + str(error)), 400

    @app.errorhandler(HTTPException)
    def http_error(error):
        return jsonify(error=error.description), error.code

    @app.get('/')
    def home():
        return send_from_directory(here / 'static', 'index.html')

    @app.get('/api/config')
    def configuration():
        return jsonify(Context(controller_path, dynamics_path).metadata())

    @app.post('/api/analyze')
    def analyze():
        ctx = Context(controller_path, dynamics_path)
        body = request.get_json()
        candidate = simulate(ctx, body)
        baseline_body = {**candidate['settings'], 'kp': ctx.gains['k_p_speed'], 'ki': ctx.gains['k_i_speed']}
        baseline = simulate(ctx, baseline_body)
        return jsonify(candidate=candidate, baseline=baseline)

    @app.post('/api/sweep')
    def screen():
        return jsonify(sweep(Context(controller_path, dynamics_path), request.get_json()))

    @app.post('/api/export')
    def export():
        return jsonify(export_candidate(Context(controller_path, dynamics_path), request.get_json(), output))

    return app
