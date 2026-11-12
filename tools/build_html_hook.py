Import("env")
import gzip
import hashlib
import os
import re
import subprocess
import sys

def _header_matches_html(html_path, header_path):
    if not os.path.exists(header_path):
        return False
    try:
        with open(html_path, "rb") as f:
            html_sha = hashlib.sha256(f.read()).hexdigest()
        with open(header_path, "r", encoding="utf-8") as f:
            hdr = f.read()
        raw = bytes(int(x, 16) for x in re.findall(r"0x([0-9A-Fa-f]{2})", hdr))
        return hashlib.sha256(gzip.decompress(raw)).hexdigest() == html_sha
    except Exception:
        return False

def build_html_header(*args, **kwargs):
    project_dir = env.get("PROJECT_DIR")
    script_path = os.path.join(project_dir, "tools", "compress_html.py")
    html_path = os.path.join(project_dir, "data", "index.html")
    header_path = os.path.join(project_dir, "src", "eq_html.h")

    if not _header_matches_html(html_path, header_path):
        print(f"[HTML PRE-BUILD] Updating {header_path} from {html_path}...")
        subprocess.check_call([sys.executable, script_path], cwd=project_dir)
    else:
        print("[HTML PRE-BUILD] src/eq_html.h is up to date.")

build_html_header()
