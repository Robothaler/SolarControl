"""
PlatformIO pre-build hook.
Runs compress_ui.py before compilation to generate index_html_gz.h
from src/web_ui.html. Also adds the generated directory to CPPPATH.
"""
Import("env")  # noqa: F821  (PlatformIO injects this)

import os
import subprocess
import sys

project_dir = env.subst("$PROJECT_DIR")   # noqa: F821

html_src  = os.path.join(project_dir, "src",     "web_ui.html")
# Output directly into include/ which is already on the compiler include path
out_file  = os.path.join(project_dir, "include", "index_html_gz.h")
script    = os.path.join(project_dir, "scripts",  "compress_ui.py")

if os.path.exists(html_src):
    result = subprocess.run(
        [sys.executable, script, html_src, out_file],
        capture_output=False,
    )
    if result.returncode != 0:
        print(f"[WebUI] ERROR: compress_ui.py exited with code {result.returncode}")
else:
    print(f"[WebUI] WARNING: {html_src} not found — skipping compression.")
    # Create an empty header so compilation does not fail
    if not os.path.exists(out_file):
        with open(out_file, "w") as f:
            f.write("#pragma once\n")
            f.write("static const uint8_t  index_html_gz[]    = {};\n")
            f.write("static const size_t   index_html_gz_len  = 0;\n")
