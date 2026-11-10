Import("env")
import os
import subprocess
import sys

def build_html_header(*args, **kwargs):
    project_dir = env.get("PROJECT_DIR")
    script_path = os.path.join(project_dir, "tools", "compress_html.py")
    html_path = os.path.join(project_dir, "data", "index.html")
    header_path = os.path.join(project_dir, "src", "eq_html.h")
    
    # Check if header needs update
    if not os.path.exists(header_path) or os.path.getmtime(html_path) > os.path.getmtime(header_path):
        print(f"[HTML PRE-BUILD] Updating {header_path} from {html_path}...")
        subprocess.check_call([sys.executable, script_path], cwd=project_dir)
    else:
        print("[HTML PRE-BUILD] src/eq_html.h is up to date.")

build_html_header()
