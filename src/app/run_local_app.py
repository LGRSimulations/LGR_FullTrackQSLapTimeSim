import argparse
import threading
import time
import webbrowser
from pathlib import Path

import uvicorn


THIS_DIR = Path(__file__).resolve().parent
SRC_DIR = THIS_DIR.parent
import sys

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from app.web import create_app


def _open_browser(host: str, port: int, delay_s: float = 1.0) -> None:
    time.sleep(delay_s)
    webbrowser.open(f"http://{host}:{port}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run LGR Sim Workbench local app")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    app = create_app()

    if not args.no_browser:
        thread = threading.Thread(target=_open_browser, args=(args.host, args.port), daemon=True)
        thread.start()

    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
