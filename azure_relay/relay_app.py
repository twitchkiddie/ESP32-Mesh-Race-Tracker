"""
ESP32 Mesh V2 - Azure Cloud Relay

Tiny Flask app that:
  1. Accepts pushed state from the local mesh_server via POST /push
  2. Serves that state at GET /api/state for the spectator page to poll
  3. Renders a read-only spectator dashboard at GET /

Deploy to Azure App Service Free tier (F1):
    cd azure_relay
    az webapp up --name <your-app-name> --runtime PYTHON:3.11 --sku F1

Set environment variables in Azure:
    az webapp config appsettings set --name <your-app-name> \\
        --resource-group <rg> \\
        --settings CLOUD_SECRET=<same-secret-as-local-server>

Optional:
    GMAPS_KEY=<Google Maps API key>   (for satellite map on spectator page)

Local dev:
    CLOUD_SECRET=dev python relay_app.py
    # Push from another terminal:
    curl -X POST http://localhost:8000/push \\
         -H "Content-Type: application/json" \\
         -H "X-Cloud-Secret: dev" \\
         -d '{"nodes":{},"gateway":{"connected":false},"timestamp":0}'
"""

import os
import time
import threading

from flask import Flask, jsonify, render_template, request, abort

# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------

app = Flask(__name__, template_folder='templates')

# ---------------------------------------------------------------------------
# In-memory state store
# ---------------------------------------------------------------------------

_state: dict   = {}     # last full state dict received from local server
_state_ts: float = 0.0  # Unix timestamp of last successful push
_lock = threading.Lock()

STALE_S   = 30   # seconds after which state is considered stale
_secret   = os.environ.get('CLOUD_SECRET', '')

# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------


@app.route('/')
def index():
    """Spectator dashboard — read-only race view."""
    gmaps_key = os.environ.get('GMAPS_KEY', '')
    return render_template('spectator.html', gmaps_key=gmaps_key)


@app.route('/push', methods=['POST'])
def push():
    """
    Receive a state push from the local mesh_server.
    Requires X-Cloud-Secret header to match CLOUD_SECRET env var.
    """
    if _secret and request.headers.get('X-Cloud-Secret', '') != _secret:
        abort(403)

    data = request.get_json(silent=True, force=True)
    if not isinstance(data, dict):
        abort(400)

    with _lock:
        global _state, _state_ts
        _state    = data
        _state_ts = time.time()

    return jsonify({'ok': True})


@app.route('/api/state')
def api_state():
    """
    Current state for the spectator page to poll.
    Augments the stored state with cloud relay metadata:
        cloud_age_s   — seconds since last push
        cloud_stale   — True if no push received for > STALE_S seconds
    """
    with _lock:
        if _state_ts:
            age   = round(time.time() - _state_ts, 1)
            stale = age > STALE_S
        else:
            age   = None
            stale = True  # no data yet → stale

        out = dict(_state)  # shallow copy is fine (nested dicts are read-only here)

    out['cloud_age_s'] = age
    out['cloud_stale'] = stale
    return jsonify(out)


@app.route('/health')
def health():
    """Azure health-check / uptime probe."""
    with _lock:
        age = round(time.time() - _state_ts, 1) if _state_ts else None
    return jsonify({'ok': True, 'state_age_s': age})


# ---------------------------------------------------------------------------
# Entry point (local dev only — Azure uses gunicorn)
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 8000))
    print(f'[Relay] Listening on http://0.0.0.0:{port}')
    print(f'[Relay] CLOUD_SECRET {"set" if _secret else "NOT SET — /push is open!"}')
    app.run(host='0.0.0.0', port=port, debug=False, threaded=True)
