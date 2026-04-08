import sys
from pathlib import Path

_root = Path(__file__).resolve().parent.parent
_src = _root / "src"
_base_src = _root.parent / "lucid-component-base" / "src"

for path in (_base_src, _src):
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))
