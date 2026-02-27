"""
ESP32 Mesh V2 - Race Dashboard Server
Plain Flask server. Reads from gateway via SerialBridge,
serves state via polling REST API (no WebSocket dependency).

Usage:
    python mesh_server.py --port COM7 --gmaps-key YOUR_KEY
    python mesh_server.py --port COM7   # Google Maps disabled, OSM fallback
"""

import argparse
import logging
import os
from pathlib import Path

from flask import Flask, render_template, jsonify, request

from serial_bridge import SerialBridge
from race_engine import RaceEngine, Course
from cloud_push import CloudPush

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(name)s %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger(__name__)

app = Flask(__name__, template_folder='templates', static_folder='static')
app.config['TEMPLATES_AUTO_RELOAD'] = True   # re-read templates on every request (dev convenience)

# Global bridge instance and config
bridge    = None   # type: SerialBridge
gmaps_key = ''     # type: str
engine    = RaceEngine()
_config_path = str(Path(__file__).parent / 'race_config.json')


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

@app.route('/')
def index():
    return render_template('dashboard.html', gmaps_key=gmaps_key)


@app.route('/api/state')
def api_state():
    """Full current state snapshot — polled by dashboard every 2 s."""
    if bridge is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    return jsonify(bridge.get_state())


@app.route('/api/nodes')
def api_nodes():
    if bridge is None:
        return jsonify({}), 503
    return jsonify(bridge.get_state()['nodes'])


@app.route('/api/gateway')
def api_gateway():
    if bridge is None:
        return jsonify({}), 503
    return jsonify(bridge.get_state()['gateway'])


@app.route('/admin')
def admin():
    return render_template('admin.html')


@app.route('/api/node/<path:mac>/name', methods=['POST', 'DELETE'])
def api_node_name(mac):
    """Set or clear a custom display name for a node."""
    if request.method == 'DELETE':
        bridge.clear_name(mac)
        return jsonify({'ok': True})
    data = request.get_json(silent=True) or {}
    name = str(data.get('name', '')).strip()
    if name:
        bridge.set_name(mac, name)
    else:
        bridge.clear_name(mac)
    return jsonify({'ok': True, 'name': name})


@app.route('/api/node/<path:mac>/id', methods=['POST'])
def api_node_set_id(mac):
    """Send CMD_SET_DEVICE_ID to a node over the mesh (via gateway)."""
    if bridge is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    data = request.get_json(silent=True) or {}
    try:
        new_id = int(data.get('id', -1))
    except (TypeError, ValueError):
        return jsonify({'error': 'Invalid id'}), 400
    if not (0 <= new_id <= 30):
        return jsonify({'error': 'ID must be 0–30'}), 400
    ok = bridge.send_cmd_set_device_id(mac, new_id)
    return jsonify({'ok': ok})


@app.route('/api/log')
def api_log():
    if bridge is None:
        return jsonify([]), 503
    limit = request.args.get('limit', 100, type=int)
    return jsonify(bridge.get_log(limit))


@app.route('/api/config')
def api_config():
    return jsonify({
        'has_gmaps': bool(gmaps_key),
        'gmaps_key': gmaps_key,
        'gateway_port': bridge.port if bridge else '',
    })


# ---------------------------------------------------------------------------
# Race engine routes
# ---------------------------------------------------------------------------

@app.route('/race')
def race():
    return render_template('race.html')


@app.route('/api/race')
def api_race():
    nodes = bridge.get_state()['nodes'] if bridge else {}
    return jsonify(engine.get_race_state(nodes))


@app.route('/api/race/courses')
def api_race_courses():
    """List all saved courses."""
    return jsonify(engine.list_courses())


@app.route('/api/race/course', methods=['GET'])
def api_race_course_get():
    c = engine.get_course()
    return jsonify(c.to_dict() if c else None)


@app.route('/api/race/course', methods=['POST'])
def api_race_course_set():
    data = request.get_json(silent=True)
    if not data:
        return jsonify({'error': 'No JSON body'}), 400
    try:
        course = Course.from_dict(data)
        engine.set_course(course)
        engine.save_config(_config_path)
        return jsonify({'ok': True})
    except Exception as e:
        return jsonify({'error': str(e)}), 400


@app.route('/api/race/course/<string:name>/activate', methods=['POST'])
def api_race_course_activate(name):
    if engine.activate_course(name):
        engine.save_config(_config_path)
        return jsonify({'ok': True})
    return jsonify({'error': 'Course not found'}), 404


@app.route('/api/race/course/<string:name>', methods=['DELETE'])
def api_race_course_delete(name):
    engine.delete_course(name)
    engine.save_config(_config_path)
    return jsonify({'ok': True})


@app.route('/api/race/reset', methods=['POST'])
def api_race_reset():
    engine.reset()
    return jsonify({'ok': True})


@app.route('/api/race/dnf/<path:mac>', methods=['POST'])
def api_race_dnf(mac):
    engine.flag_dnf(mac)
    return jsonify({'ok': True})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    global bridge, gmaps_key

    parser = argparse.ArgumentParser(description='ESP32 Mesh V2 Race Dashboard')
    parser.add_argument('--port', default='COM7',
                        help='Gateway serial port (default: COM7)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Serial baud rate (default: 115200)')
    parser.add_argument('--host', default='0.0.0.0',
                        help='Web server host (default: 0.0.0.0)')
    parser.add_argument('--web-port', type=int, default=5000,
                        help='Web server port (default: 5000)')
    parser.add_argument('--gmaps-key', default=os.environ.get('GMAPS_KEY', ''),
                        help='Google Maps API key (or set GMAPS_KEY env var)')
    parser.add_argument('--cloud-url', default=os.environ.get('CLOUD_URL', ''),
                        help='Azure relay URL, e.g. https://myapp.azurewebsites.net (or set CLOUD_URL env var)')
    parser.add_argument('--cloud-secret', default=os.environ.get('CLOUD_SECRET', ''),
                        help='Shared secret for relay push endpoint (or set CLOUD_SECRET env var)')
    args = parser.parse_args()

    gmaps_key = args.gmaps_key
    if gmaps_key:
        logger.info('[Server] Google Maps enabled')
    else:
        logger.info('[Server] No Google Maps key — using OpenStreetMap (Leaflet)')

    # Load saved race course (non-fatal if missing)
    if engine.load_config(_config_path):
        logger.info('[Server] Race course loaded from %s', _config_path)
    else:
        logger.info('[Server] No saved race course — set one via /race')

    # Start serial bridge in background thread
    bridge = SerialBridge(port=args.port, baud=args.baud)
    bridge.on_update(lambda nodes, gw: engine.update(nodes))
    bridge.start()

    # Optional cloud relay push (disabled if --cloud-url / CLOUD_URL not set)
    # Push includes full node state + race standings + course so the
    # spectator page can show arrow markers, start/finish lines, and rankings.
    def get_cloud_state():
        state = bridge.get_state()
        state['race'] = engine.get_race_state(state['nodes'])
        return state

    cloud = CloudPush(url=args.cloud_url, secret=args.cloud_secret)
    cloud.set_state_fn(get_cloud_state)
    cloud.start()

    logger.info('[Server] Dashboard at http://localhost:%d/', args.web_port)
    logger.info('[Server] LAN access:  http://0.0.0.0:%d/', args.web_port)
    logger.info('[Server] Dashboard polls /api/state every 2 s (no WebSocket needed)')

    # Run plain threaded Flask (blocking)
    app.run(
        host=args.host,
        port=args.web_port,
        debug=False,
        threaded=True,
        use_reloader=False,
    )

    cloud.stop()
    bridge.stop()


if __name__ == '__main__':
    main()
