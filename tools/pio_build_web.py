# PlatformIO pre-build hook: rebuild src/web_assets.h when anything in web/
# (or the builder itself) is newer than the generated header.
import pathlib
import subprocess
import sys

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

root = pathlib.Path(env["PROJECT_DIR"])  # noqa: F821
out = root / "src" / "web_assets.h"
sources = [p for p in (root / "web").rglob("*") if p.is_file()] + [root / "tools" / "build_web.py"]
if not out.exists() or max(p.stat().st_mtime for p in sources) > out.stat().st_mtime:
    subprocess.check_call([sys.executable, str(root / "tools" / "build_web.py")])
