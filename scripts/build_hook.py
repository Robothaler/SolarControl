"""
PlatformIO pre-build hook.

1. Runs compress_ui.py before compilation to generate index_html_gz.h
   from src/web_ui.html.

2. Patches managed_components/espressif__esp_insights/CMakeLists.txt to
   fix a PlatformIO build bug: target_add_binary_data() generates a doubled
   build path (.pio/build/.../.pio/build/.../https_server.crt.S.o) which
   causes the build to fail. The fix wraps the call in
   if(CONFIG_ESP_INSIGHTS_ENABLED) so it is skipped when Insights is
   disabled (CONFIG_ESP_INSIGHTS_ENABLED=n in sdkconfig.defaults).

   After patching, CHECKSUMS.json is updated with the new SHA256 hash so
   the IDF component manager does not detect the modification and
   re-download the original (unpatched) component.

   This patch is re-applied automatically after every fresh clone /
   component manager re-download.
"""
Import("env")  # noqa: F821  (PlatformIO injects this)

import hashlib
import json
import os
import subprocess
import sys

project_dir = env.subst("$PROJECT_DIR")   # noqa: F821

# ── 1. WebUI HTML komprimieren ──────────────────────────────────────────────
html_src  = os.path.join(project_dir, "src",     "web_ui.html")
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
    if not os.path.exists(out_file):
        with open(out_file, "w") as f:
            f.write("#pragma once\n")
            f.write("static const uint8_t  index_html_gz[]    = {};\n")
            f.write("static const size_t   index_html_gz_len  = 0;\n")

# ── 2. esp_insights CMakeLists.txt patchen ──────────────────────────────────
# target_add_binary_data() erzeugt in PlatformIO einen verdoppelten Pfad.
# Fix: Cert-Embedding in if(CONFIG_ESP_INSIGHTS_ENABLED) einwickeln.
# Damit wird es übersprungen wenn CONFIG_ESP_INSIGHTS_ENABLED=n.
#
# Der IDF Component Manager prüft SHA256-Hashes aller Dateien gegen
# CHECKSUMS.json. Nach dem Patchen wird CHECKSUMS.json mit dem neuen
# Hash aktualisiert, damit der Component Manager kein Re-Download auslöst.

_INSIGHTS_DIR    = os.path.join(project_dir, "managed_components", "espressif__esp_insights")
_INSIGHTS_CMAKE  = os.path.join(_INSIGHTS_DIR, "CMakeLists.txt")
_INSIGHTS_CHECKS = os.path.join(_INSIGHTS_DIR, "CHECKSUMS.json")

_ORIGINAL = (
    "if(CONFIG_ESP_INSIGHTS_TRANSPORT_MQTT)\n"
    "    target_add_binary_data(${COMPONENT_TARGET} \"server_certs/mqtt_server.crt\" TEXT)\n"
    "    target_sources(${COMPONENT_LIB} PRIVATE \"src/transport/esp_insights_mqtt.c\")\n"
    "else()\n"
    "    target_add_binary_data(${COMPONENT_TARGET} \"server_certs/https_server.crt\" TEXT)\n"
    "    idf_component_get_property(http_client_lib esp_http_client COMPONENT_LIB)\n"
    "    target_link_libraries(${COMPONENT_LIB} PRIVATE ${http_client_lib})\n"
    "    target_sources(${COMPONENT_LIB} PRIVATE \"src/transport/esp_insights_https.c\")\n"
    "endif()"
)

_PATCHED = (
    "# target_add_binary_data causes a doubled build-path in PlatformIO.\n"
    "# Only embed certs when Insights is actually enabled (sdkconfig.defaults).\n"
    "if(CONFIG_ESP_INSIGHTS_ENABLED)\n"
    "    if(CONFIG_ESP_INSIGHTS_TRANSPORT_MQTT)\n"
    "        target_add_binary_data(${COMPONENT_TARGET} \"server_certs/mqtt_server.crt\" TEXT)\n"
    "        target_sources(${COMPONENT_LIB} PRIVATE \"src/transport/esp_insights_mqtt.c\")\n"
    "    else()\n"
    "        target_add_binary_data(${COMPONENT_TARGET} \"server_certs/https_server.crt\" TEXT)\n"
    "        idf_component_get_property(http_client_lib esp_http_client COMPONENT_LIB)\n"
    "        target_link_libraries(${COMPONENT_LIB} PRIVATE ${http_client_lib})\n"
    "        target_sources(${COMPONENT_LIB} PRIVATE \"src/transport/esp_insights_https.c\")\n"
    "    endif()\n"
    "endif()"
)


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def _update_checksums(cmake_path, checksums_path):
    """Aktualisiert den SHA256-Hash von CMakeLists.txt in CHECKSUMS.json."""
    if not os.path.exists(checksums_path):
        return
    with open(checksums_path, "r") as f:
        data = json.load(f)
    new_hash = _sha256(cmake_path)
    new_size = os.path.getsize(cmake_path)
    changed  = False
    for entry in data.get("files", []):
        if entry.get("path") == "CMakeLists.txt":
            if entry["hash"] != new_hash:
                entry["hash"] = new_hash
                entry["size"] = new_size
                changed = True
            break
    if changed:
        with open(checksums_path, "w") as f:
            json.dump(data, f, indent=2)
        print("[Hook] CHECKSUMS.json aktualisiert (neuer CMakeLists.txt-Hash)")


def _read_cmake_cache(cmake_cache_path):
    """Liest CMakeCache.txt und gibt ein Dict der Einträge zurück."""
    result = {}
    if not os.path.exists(cmake_cache_path):
        return result
    try:
        with open(cmake_cache_path, "r") as f:
            for line in f:
                line = line.strip()
                if line.startswith("#") or line.startswith("//") or "=" not in line:
                    continue
                key_type, _, value = line.partition("=")
                key = key_type.split(":")[0]
                result[key] = value
    except Exception:
        pass
    return result


def _cmake_regenerate(build_dir, project_dir):
    """Ruft cmake auf um build.ninja neu zu generieren.

    Nötig wenn CMakeLists.txt erst NACH dem cmake-Konfigurationsschritt gepatcht
    wurde (pioarduino führt cmake VOR den extra_scripts aus).
    Leitet IDF_PATH und Toolchain-Pfad aus CMakeCache.txt ab.
    """
    cmake_bin = os.path.join(
        os.path.expanduser("~"),
        ".platformio", "packages", "tool-cmake", "bin", "cmake",
    )
    if not os.path.exists(cmake_bin):
        import shutil
        cmake_bin = shutil.which("cmake") or cmake_bin
    if not os.path.exists(cmake_bin):
        print(f"[Hook] WARNUNG: cmake nicht gefunden — cmake-Regenerierung übersprungen")
        return

    cmake_cache_path = os.path.join(build_dir, "CMakeCache.txt")
    cache = _read_cmake_cache(cmake_cache_path)

    # IDF_PATH aus CMAKE_TOOLCHAIN_FILE ableiten
    # z.B. /.../framework-espidf/tools/cmake/toolchain-esp32s3.cmake → /.../framework-espidf
    toolchain_file = cache.get("CMAKE_TOOLCHAIN_FILE", "")
    idf_path = ""
    idx = toolchain_file.find("/tools/cmake/")
    if idx >= 0:
        idf_path = toolchain_file[:idx]

    if not idf_path or not os.path.isdir(idf_path):
        print(f"[Hook] WARNUNG: IDF_PATH nicht ableitbar ({idf_path!r}) — cmake-Regenerierung übersprungen")
        return

    # Toolchain-Bin-Dir aus einem der AR-Pfade ableiten
    # z.B. /.../toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc-ar
    toolchain_bin = ""
    for key in ("CMAKE_ASM_COMPILER_AR", "CMAKE_C_COMPILER_AR", "CMAKE_CXX_COMPILER_AR"):
        val = cache.get(key, "")
        if val and os.path.isfile(val):
            toolchain_bin = os.path.dirname(val)
            break

    env = os.environ.copy()
    env["IDF_PATH"] = idf_path
    if toolchain_bin:
        env["PATH"] = toolchain_bin + os.pathsep + env.get("PATH", "")

    print(f"[Hook] cmake-Regenerierung  IDF_PATH={idf_path}")
    result = subprocess.run(
        [cmake_bin, "-S", project_dir, "-B", build_dir],
        env=env,
        capture_output=False,
    )
    if result.returncode == 0:
        print("[Hook] cmake-Regenerierung erfolgreich — build.ninja aktualisiert")
    else:
        print(f"[Hook] WARNUNG: cmake-Regenerierung fehlgeschlagen (code {result.returncode})")


_build_dir = env.subst("$BUILD_DIR")   # noqa: F821
_build_ninja = os.path.join(_build_dir, "build.ninja")

if os.path.exists(_INSIGHTS_CMAKE):
    with open(_INSIGHTS_CMAKE, "r") as f:
        content = f.read()

    if _ORIGINAL in content:
        content = content.replace(_ORIGINAL, _PATCHED)
        with open(_INSIGHTS_CMAKE, "w") as f:
            f.write(content)
        _update_checksums(_INSIGHTS_CMAKE, _INSIGHTS_CHECKS)
        print("[Hook] esp_insights CMakeLists.txt gepatcht (target_add_binary_data Fix)")
        # build.ninja wurde vor dem Patch generiert → cmake muss neu laufen
        if os.path.exists(_build_ninja):
            _cmake_regenerate(_build_dir, project_dir)
    elif _PATCHED in content:
        # Bereits gepatcht – CHECKSUMS.json prüfen und ggf. cmake regenerieren
        # wenn build.ninja älter ist als CMakeLists.txt (z.B. nach Cache-Löschung)
        _update_checksums(_INSIGHTS_CMAKE, _INSIGHTS_CHECKS)
        print("[Hook] esp_insights CMakeLists.txt bereits gepatcht — OK")
        if os.path.exists(_build_ninja):
            ninja_mtime  = os.path.getmtime(_build_ninja)
            cmake_mtime  = os.path.getmtime(_INSIGHTS_CMAKE)
            if cmake_mtime > ninja_mtime:
                print("[Hook] build.ninja ist älter als CMakeLists.txt — cmake-Regenerierung nötig")
                _cmake_regenerate(_build_dir, project_dir)
    else:
        print("[Hook] WARNUNG: esp_insights CMakeLists.txt hat unerwartetes Format — Patch übersprungen")
else:
    print("[Hook] esp_insights CMakeLists.txt nicht gefunden — wird beim ersten Build heruntergeladen")

# ── 3. esp_rainmaker Zertifikate vorab generieren ──────────────────────────
# target_add_binary_data() erzeugt cmake CUSTOM_COMMANDs um .crt → .S zu
# konvertieren. PlatformIO/pioarduino führt diese Commands NIE aus (SCons
# kompiliert direkt aus dem cmake-Codemodel, ohne Ninja/cmake zu starten).
# Fix: .S-Dateien im Build-Verzeichnis vorab per data_file_embed_asm.cmake
# generieren, damit SCons sie direkt findet.
#
# Außerdem: pioarduino verdoppelt den Pfad (.pio/build/…/.pio/build/…/foo.S.o)
# → Das Zielverzeichnis für .o-Dateien muss ebenfalls existieren.

_RMAKER_DIR = os.path.join(project_dir, "managed_components", "espressif__esp_rainmaker")
_RMAKER_CERTS_DIR = os.path.join(_RMAKER_DIR, "server_certs")
_RMAKER_CERTS = [
    "rmaker_mqtt_server.crt",
    "rmaker_claim_service_server.crt",
    "rmaker_ota_server.crt",
]

if os.path.isdir(_RMAKER_CERTS_DIR):
    # IDF_PATH aus CMakeCache.txt ableiten (bereits in _build_dir verfügbar)
    _cache = _read_cmake_cache(os.path.join(_build_dir, "CMakeCache.txt"))
    _toolchain_file = _cache.get("CMAKE_TOOLCHAIN_FILE", "")
    _idf_path = ""
    _idx = _toolchain_file.find("/tools/cmake/")
    if _idx >= 0:
        _idf_path = _toolchain_file[:_idx]

    _embed_script = os.path.join(_idf_path, "tools", "cmake", "scripts", "data_file_embed_asm.cmake") if _idf_path else ""

    _cmake_bin = os.path.join(
        os.path.expanduser("~"),
        ".platformio", "packages", "tool-cmake", "bin", "cmake",
    )
    if not os.path.exists(_cmake_bin):
        import shutil as _shutil
        _cmake_bin = _shutil.which("cmake") or _cmake_bin

    # Verdoppeltes Verzeichnis erstellen: BUILD_DIR/rel_build_dir/
    _rel_build = os.path.relpath(_build_dir, project_dir)   # e.g. .pio/build/matter_serial
    _doubled_dir = os.path.join(_build_dir, _rel_build)
    os.makedirs(_doubled_dir, exist_ok=True)

    if not _idf_path:
        print("[Hook] WARNUNG: IDF_PATH nicht ableitbar — esp_rainmaker Zertifikat-Generierung übersprungen")
    elif not os.path.exists(_embed_script):
        print(f"[Hook] WARNUNG: data_file_embed_asm.cmake nicht gefunden ({_embed_script}) — übersprungen")
    elif not os.path.exists(_cmake_bin):
        print(f"[Hook] WARNUNG: cmake nicht gefunden — esp_rainmaker Zertifikat-Generierung übersprungen")
    else:
        for _cert_name in _RMAKER_CERTS:
            _cert_file = os.path.join(_RMAKER_CERTS_DIR, _cert_name)
            _s_file    = os.path.join(_build_dir, _cert_name + ".S")
            if not os.path.exists(_cert_file):
                print(f"[Hook] WARNUNG: {_cert_name} nicht gefunden — übersprungen")
                continue
            if os.path.exists(_s_file) and os.path.getmtime(_s_file) >= os.path.getmtime(_cert_file):
                print(f"[Hook] {_cert_name}.S aktuell — übersprungen")
                continue
            _result = subprocess.run(
                [
                    _cmake_bin,
                    f"-DDATA_FILE={_cert_file}",
                    f"-DSOURCE_FILE={_s_file}",
                    "-DFILE_TYPE=TEXT",
                    "-P", _embed_script,
                ],
                capture_output=True,
                text=True,
            )
            if _result.returncode == 0:
                print(f"[Hook] {_cert_name}.S generiert → {_s_file}")
            else:
                print(f"[Hook] FEHLER: {_cert_name}.S Generierung fehlgeschlagen (code {_result.returncode})")
                if _result.stderr:
                    print(_result.stderr.strip())
else:
    print("[Hook] esp_rainmaker server_certs nicht gefunden — wird beim ersten Build heruntergeladen")
