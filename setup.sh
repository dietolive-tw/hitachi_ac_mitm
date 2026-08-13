#!/usr/bin/env bash
# Setup script for hitachi_ac_mitm standalone repo.
#
# Prerequisites:
#   - ESP-IDF v5.5+ installed (e.g. ~/works/esp32/esp-idf)
#   - Both ESP-IDF and esp-matter env sourced before building:
#       . ~/works/esp32/esp-idf/export.sh
#       . <esp-matter-path>/export.sh
#
# What this script does:
#   1. Ensures esp-matter is cloned (or uses ESP_MATTER_PATH if set)
#   2. Applies pending SDK patches from patches/ to esp-matter
#   3. Symlinks this repo into esp-matter/examples/hitachi_ac_mitm so the
#      standard `idf.py build` flow works as if it lived in the SDK tree.
#
# After running, build with:
#   cd $ESP_MATTER_PATH/examples/hitachi_ac_mitm
#   idf.py -DIDF_TARGET=esp32c6 build
#   idf.py -p /dev/ttyACM0 flash monitor

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Resolve esp-matter path: prefer $ESP_MATTER_PATH, then $WORKSPACE, then default.
ESP_MATTER_PATH="${ESP_MATTER_PATH:-${WORKSPACE:-}}"
if [[ -z "$ESP_MATTER_PATH" ]]; then
    ESP_MATTER_PATH="$HOME/works/esp32/dev/esp-matter"
fi

if [[ ! -e "$ESP_MATTER_PATH/.git" ]]; then
    echo "[setup] esp-matter not found at $ESP_MATTER_PATH"
    echo "[setup] Cloning espressif/esp-matter into $ESP_MATTER_PATH ..."
    mkdir -p "$(dirname "$ESP_MATTER_PATH")"
    git clone --recursive https://github.com/espressif/esp-matter.git "$ESP_MATTER_PATH"
fi

echo "[setup] Using esp-matter at: $ESP_MATTER_PATH"

# Apply pending SDK patches
PATCH_DIR="$SCRIPT_DIR/patches"
if [[ -d "$PATCH_DIR" ]]; then
    echo "[setup] Applying SDK patches from $PATCH_DIR ..."
    for patch in "$PATCH_DIR"/esp-matter-*.patch; do
        [[ -e "$patch" ]] || continue
        name=$(basename "$patch")
        if git -C "$ESP_MATTER_PATH" apply --check "$patch" 2>/dev/null; then
            git -C "$ESP_MATTER_PATH" apply "$patch"
            echo "[setup]   applied: $name"
        elif git -C "$ESP_MATTER_PATH" apply --reverse --check "$patch" 2>/dev/null; then
            echo "[setup]   already applied, skipping: $name"
        else
            echo "[setup]   FAILED to apply (conflict): $name"
            echo "[setup]   Inspect with: git -C \"$ESP_MATTER_PATH\" apply --check \"$patch\""
            exit 1
        fi
    done
fi

# Symlink this repo into esp-matter/examples/hitachi_ac_mitm
EXAMPLE_LINK="$ESP_MATTER_PATH/examples/hitachi_ac_mitm"
if [[ -L "$EXAMPLE_LINK" ]]; then
    echo "[setup] Symlink already exists: $EXAMPLE_LINK -> $(readlink "$EXAMPLE_LINK")"
elif [[ -e "$EXAMPLE_LINK" ]]; then
    echo "[setup] WARNING: $EXAMPLE_LINK exists and is not a symlink. Leaving it as-is."
    echo "[setup]   Remove it manually if you want this repo to take its place:"
    echo "[setup]   rm -rf \"$EXAMPLE_LINK\""
else
    ln -s "$SCRIPT_DIR" "$EXAMPLE_LINK"
    echo "[setup] Symlinked $EXAMPLE_LINK -> $SCRIPT_DIR"
fi

cat <<EOF

[setup] Done. Next steps:

  # 1. source ESP-IDF + esp-matter env (once per shell)
  . ~/works/esp32/esp-idf/export.sh
  . "$ESP_MATTER_PATH/export.sh"

  # 2. build / flash / monitor
  cd "$EXAMPLE_LINK"
  idf.py -DIDF_TARGET=esp32c6 build
  idf.py -p /dev/ttyACM0 flash monitor

  # OTA version override (optional)
  idf.py -DIDF_TARGET=esp32c6 -DPROJECT_VER_NUMBER=2 -DPROJECT_VER=2.0 build
EOF
