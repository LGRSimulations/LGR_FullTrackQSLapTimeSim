import json
import re
import sys
from pathlib import Path

import httpx

LESSONS_DIR = Path(__file__).resolve().parents[1] / "docs" / "lessons"
OUT_PATH = LESSONS_DIR / "index.json"
DEEPSEEK_URL = "https://api.deepseek.com/chat/completions"
DEEPSEEK_MODEL = "deepseek-chat"
SKIP_FILES = {"README.md"}


def split_sections(text: str, filename: str) -> list[dict]:
    sections = []
    heading = None
    level = 0
    buf: list[str] = []

    for line in text.splitlines():
        m = re.match(r'^(#{1,3})\s+(.+)', line)
        if m:
            if heading:
                sections.append({
                    "file": filename,
                    "section": heading,
                    "heading_level": level,
                    "content": "\n".join(buf).strip(),
                })
            heading = m.group(2).strip()
            level = len(m.group(1))
            buf = []
        else:
            buf.append(line)

    if heading:
        sections.append({
            "file": filename,
            "section": heading,
            "heading_level": level,
            "content": "\n".join(buf).strip(),
        })

    return [s for s in sections if s["content"]]


def summarize(section: dict, api_key: str) -> str:
    prompt = (
        "Summarize this section from a Formula Student lap time simulator knowledge base "
        "in one sentence. Be specific about what the section covers. "
        "Do not start with 'This section'.\n\n"
        f"Section: {section['section']}\n\n{section['content'][:1500]}"
    )
    resp = httpx.post(
        DEEPSEEK_URL,
        headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
        json={
            "model": DEEPSEEK_MODEL,
            "messages": [{"role": "user", "content": prompt}],
            "max_tokens": 80,
        },
        timeout=30,
    )
    resp.raise_for_status()
    return resp.json()["choices"][0]["message"]["content"].strip()


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python tools/build_lesson_index.py <deepseek-api-key>")
        sys.exit(1)
    api_key = sys.argv[1]

    entries = []
    for md_file in sorted(LESSONS_DIR.glob("*.md")):
        if md_file.name in SKIP_FILES:
            continue
        print(f"Processing {md_file.name}...")
        for sec in split_sections(md_file.read_text(encoding="utf-8"), md_file.name):
            print(f"  -> {sec['section']}")
            summary = summarize(sec, api_key)
            entries.append({
                "file": sec["file"],
                "section": sec["section"],
                "summary": summary,
                "heading_level": sec["heading_level"],
            })

    OUT_PATH.write_text(json.dumps(entries, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nDone. {len(entries)} sections -> {OUT_PATH}")


if __name__ == "__main__":
    main()
