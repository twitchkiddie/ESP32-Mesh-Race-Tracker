"""
ESP32 Mesh V2 - Cloud Push

Background daemon thread that periodically POSTs the local bridge state to an
Azure (or any HTTP) relay when internet is available.

Uses stdlib only — no extra pip dependencies on the local server.

Configuration (pass to CloudPush() or set env vars before calling main()):
    CLOUD_URL     https://<your-app>.azurewebsites.net
    CLOUD_SECRET  shared secret (also set on the relay as CLOUD_SECRET env var)

Usage in mesh_server.py:
    cloud = CloudPush(url=args.cloud_url, secret=args.cloud_secret)
    cloud.set_state_fn(bridge.get_state)
    cloud.start()
"""

import json
import logging
import threading
import urllib.request
import urllib.error

logger = logging.getLogger(__name__)


class CloudPush:
    """
    Pushes the current mesh state to a remote relay endpoint every INTERVAL seconds.

    The relay is expected to accept POST /push with:
        Content-Type: application/json
        X-Cloud-Secret: <secret>
    Body: the JSON-encoded state dict from bridge.get_state().

    If the URL or secret is not set the thread never starts (safe no-op).
    If a push fails the error is logged once; recovery is automatic on next interval.
    """

    def __init__(self, url: str, secret: str, interval: int = 5):
        self._url    = url.rstrip('/') if url else ''
        self._secret = secret or ''
        self._intv   = interval
        self._fn     = None              # Callable[[], dict] — set via set_state_fn()
        self._stop   = threading.Event()
        self._ok     = False             # tracks whether last push succeeded

    def set_state_fn(self, fn):
        """Register the function that returns the current state dict."""
        self._fn = fn

    def start(self):
        """Start the background push thread. Safe to call even if not configured."""
        if not self._url or not self._secret:
            logger.info('[Cloud] Push not configured — cloud relay disabled')
            return
        logger.info('[Cloud] Push enabled → %s  interval=%ds', self._url, self._intv)
        t = threading.Thread(target=self._loop, daemon=True, name='cloud-push')
        t.start()

    def stop(self):
        """Signal the background thread to stop."""
        self._stop.set()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _loop(self):
        while not self._stop.is_set():
            try:
                self._push_once()
                if not self._ok:
                    logger.info('[Cloud] Connected to relay at %s', self._url)
                    self._ok = True
            except urllib.error.URLError as e:
                if self._ok:
                    logger.warning('[Cloud] Push failed (network): %s', e.reason)
                    self._ok = False
            except Exception as e:
                if self._ok:
                    logger.warning('[Cloud] Push failed: %s', e)
                    self._ok = False
            self._stop.wait(self._intv)

    def _push_once(self):
        if self._fn is None:
            return
        state = self._fn()
        body  = json.dumps(state, default=str).encode('utf-8')
        req   = urllib.request.Request(
            f'{self._url}/push',
            data=body,
            headers={
                'Content-Type':   'application/json',
                'X-Cloud-Secret': self._secret,
            },
            method='POST',
        )
        # Raises urllib.error.URLError / HTTPError on failure
        with urllib.request.urlopen(req, timeout=5) as resp:
            if resp.status not in (200, 204):
                raise RuntimeError(f'Relay returned HTTP {resp.status}')
