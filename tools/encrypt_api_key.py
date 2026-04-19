import sys
from pathlib import Path

from cryptography.fernet import Fernet

FERNET_SECRET = b"REMOVED"


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python tools/encrypt_api_key.py <deepseek-api-key>")
        sys.exit(1)
    plain = sys.argv[1].strip().encode()
    token = Fernet(FERNET_SECRET).encrypt(plain)
    out = Path(__file__).resolve().parents[1] / "config" / "deepseek_key.enc"
    out.parent.mkdir(exist_ok=True)
    out.write_bytes(token)
    print(f"Saved to {out}")


if __name__ == "__main__":
    main()
