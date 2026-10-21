"""Local web dashboard (optional --web mode). No import of tacoma_monitor -> no cycles."""
import json
import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import urlparse

log = logging.getLogger("tacoma")

_HTML_PATH = Path(__file__).resolve().parent / "tacoma_dashboard.html"
_monitor_ref: Optional[Any] = None

class _DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # quieter than default
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Cache-Control", "no-store")

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ("/", "/index.html", "/dashboard"):
            if not _HTML_PATH.exists():
                self.send_error(404, "tacoma_dashboard.html not found next to the script")
                return
            body = _HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/api/data":
            if _monitor_ref is None:
                self.send_error(503, "Monitor not ready")
                return
            try:
                payload = json.dumps(_monitor_ref.get_snapshot()).encode("utf-8")
            except Exception as e:
                log.exception("/api/data snapshot failed")
                self.send_error(500, str(e))
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self._cors()
            self.end_headers()
            self.wfile.write(payload)
            return

        self.send_error(404)


def start_web_server(monitor, port: int = 8765):
    global _monitor_ref
    _monitor_ref = monitor
    server = ThreadingHTTPServer(("127.0.0.1", port), _DashboardHandler)
    server.daemon_threads = True
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    monitor.console.print(
        f"[bold red]Web dashboard[/bold red] [yellow](TRD cluster)[/yellow]: "
        f"[bold]http://127.0.0.1:{port}/[/bold]  ·  API /api/data"
    )
    monitor.console.print(
        "[dim]Browser: S=save log  P=pause  B=baseline  ·  Autosave toggle in footer[/dim]"
    )
    return server




_start_web_server = start_web_server  # backward-compatible name
