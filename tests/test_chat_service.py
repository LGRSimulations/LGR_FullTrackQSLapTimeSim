import json
from pathlib import Path

import pytest
from cryptography.fernet import Fernet

from app.services.chat_service import FERNET_SECRET, _decrypt_key, _load_index, _load_section_content


def test_decrypt_key_roundtrip(tmp_path):
    enc_path = tmp_path / "test.enc"
    enc_path.write_bytes(Fernet(FERNET_SECRET).encrypt(b"sk-test-key-abc"))
    assert _decrypt_key(enc_path) == "sk-test-key-abc"


def test_load_index_returns_required_fields(tmp_path):
    index = [{"file": "Tyre-Model.md", "section": "Where base_mu fits", "summary": "A summary.", "heading_level": 2}]
    index_path = tmp_path / "index.json"
    index_path.write_text(json.dumps(index), encoding="utf-8")
    result = _load_index(index_path)
    assert isinstance(result, list)
    assert result[0]["file"] == "Tyre-Model.md"
    assert result[0]["section"] == "Where base_mu fits"


def test_load_section_content_returns_nonempty_text():
    content = _load_section_content("Tyre-Model.md", "Where base_mu fits")
    assert "base_mu" in content
    assert len(content) > 20
