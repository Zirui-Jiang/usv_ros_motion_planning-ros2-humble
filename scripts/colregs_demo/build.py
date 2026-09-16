"""Build the portable, offline HTML demo using only Python's standard library."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent
template = (ROOT / "page.html").read_text(encoding="utf-8")
for marker, filename in (
    ("/* INLINE_STYLE */", "style.css"),
    ("/* INLINE_ENGINE */", "engine.js"),
    ("/* INLINE_APP */", "app.js"),
):
    source = (ROOT / filename).read_text(encoding="utf-8")
    if filename.endswith(".js") and "</script" in source.lower():
        raise ValueError(f"{filename}: script closing tag cannot be embedded")
    if template.count(marker) != 1:
        raise ValueError(f"Expected one template marker: {marker}")
    template = template.replace(marker, source)
output = ROOT / "colregs-demo.html"
output.write_text(template, encoding="utf-8")
print(f"Built {output} ({output.stat().st_size:,} bytes)")
