import json
import re
from pathlib import Path

import httpx
from cryptography.fernet import Fernet

from app.paths import repo_root

FERNET_SECRET = b"REMOVED"
DEEPSEEK_URL = "https://api.deepseek.com/chat/completions"
DEEPSEEK_MODEL = "deepseek-chat"


def _key_path() -> Path:
    return repo_root() / "config" / "deepseek_key.enc"


def _index_path() -> Path:
    return repo_root() / "docs" / "lessons" / "index.json"


def _lessons_dir() -> Path:
    return repo_root() / "docs" / "lessons"


def _decrypt_key(key_path: Path | None = None) -> str:
    path = key_path or _key_path()
    return Fernet(FERNET_SECRET).decrypt(path.read_bytes()).decode()


def _load_index(index_path: Path | None = None) -> list[dict]:
    path = index_path or _index_path()
    return json.loads(path.read_text(encoding="utf-8"))


def _load_section_content(file: str, section: str) -> str:
    md = (_lessons_dir() / file).read_text(encoding="utf-8")
    in_section = False
    buf: list[str] = []
    for line in md.splitlines():
        m = re.match(r'^#{1,3}\s+(.+)', line)
        if m:
            if in_section:
                break
            if m.group(1).strip() == section:
                in_section = True
        elif in_section:
            buf.append(line)
    return "\n".join(buf).strip()


def _call_deepseek(messages: list[dict], api_key: str, max_tokens: int = 600) -> str:
    resp = httpx.post(
        DEEPSEEK_URL,
        headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
        json={"model": DEEPSEEK_MODEL, "messages": messages, "max_tokens": max_tokens},
        timeout=60,
    )
    resp.raise_for_status()
    return resp.json()["choices"][0]["message"]["content"].strip()


def chat(question: str, history: list[dict]) -> dict:
    api_key = _decrypt_key()
    index = _load_index()

    index_lines = "\n".join(
        f'- "{e["file"]} > {e["section"]}": {e["summary"]}' for e in index
    )
    retrieval_prompt = (
        "You are a retrieval filter for a Formula Student lap time simulator knowledge base.\n"
        "Given the user question and the list of available sections, return a JSON array of "
        "section identifiers in the format \"filename > section name\" that are relevant.\n"
        "Return only the JSON array, nothing else.\n\n"
        f"Sections:\n{index_lines}\n\nQuestion: {question}"
    )
    raw = _call_deepseek(
        [{"role": "user", "content": retrieval_prompt}], api_key, max_tokens=300
    )

    try:
        cleaned = re.sub(r"^```(?:json)?\s*|\s*```$", "", raw.strip())
        selected = json.loads(cleaned)
        if not isinstance(selected, list):
            raise ValueError
    except (json.JSONDecodeError, ValueError):
        selected = [f"{e['file']} > {e['section']}" for e in index]

    index_map = {f"{e['file']} > {e['section']}": e for e in index}
    matched = [index_map[k] for k in selected if k in index_map]
    if not matched:
        matched = index[:6]

    context = "\n\n---\n\n".join(
        f"## {e['file']} > {e['section']}\n{_load_section_content(e['file'], e['section'])}"
        for e in matched
    )
    system = (
        "You are a tutor for a Formula Student lap time simulator. "
        "Use the reference notes below as your primary source of truth. "
        "Reason beyond the notes when needed but say so explicitly. "
        "Be concise and precise. Do not use em dashes or semicolons.\n\n"
        f"Reference notes:\n\n{context}"
    )
    messages = [
        {"role": "system", "content": system},
        *history,
        {"role": "user", "content": question},
    ]
    answer = _call_deepseek(messages, api_key, max_tokens=600)

    return {
        "answer": answer,
        "sources": [{"file": e["file"], "section": e["section"]} for e in matched],
    }
