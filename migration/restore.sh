#!/usr/bin/env bash
#
# restore.sh — run this on the NEW machine (T14)
#
# Reads the ./migration/ folder produced by capture.sh and reinstalls
# everything. Run it from the directory that contains ./migration/.
#
# Order matters: repos + keys go back FIRST, so that apt can resolve the
# third-party packages (ROS, NVIDIA, etc.) in the main install step.

set -uo pipefail   # NOT -e: we want to keep going if one package fails

SRC="migration"

if [[ ! -d "$SRC" ]]; then
  echo "ERROR: ./$SRC/ not found. Run this from the folder containing it." >&2
  exit 1
fi

echo "==> Restoring from ./$SRC/"

# ---------------------------------------------------------------------------
# 1. Third-party repos + keys FIRST
# ---------------------------------------------------------------------------
echo "  - restoring apt sources + keys"
sudo cp -r "$SRC"/sources/sources.list.d/*  /etc/apt/sources.list.d/ 2>/dev/null || true
sudo cp -r "$SRC"/sources/keyrings/*        /etc/apt/keyrings/       2>/dev/null || true
sudo cp -r "$SRC"/sources/trusted.gpg.d/*   /etc/apt/trusted.gpg.d/  2>/dev/null || true
# (sources.list itself is left alone — the new OS install already has a good
#  default one for its release. Merge by hand only if you had custom edits.)

echo "  - apt update"
sudo apt update || true

# ---------------------------------------------------------------------------
# 2. apt packages
# ---------------------------------------------------------------------------
if [[ -f "$SRC/apt-packages.txt" ]]; then
  echo "  - installing apt packages"
  # --no-install-recommends optional; drop it if you want recommends pulled in.
  xargs -a "$SRC/apt-packages.txt" sudo apt install -y || \
    echo "    (some apt packages failed — see output above; often a renamed/absent pkg)"
fi

# ---------------------------------------------------------------------------
# 3. pip
# ---------------------------------------------------------------------------
if [[ -f "$SRC/pip-packages.txt" ]]; then
  echo "  - installing pip packages"
  PIP=pip; command -v pip >/dev/null 2>&1 || PIP=pip3
  # NOTE: this is system-wide pip. If your robot code lives in a venv/conda env,
  # skip this and recreate that env from its own requirements instead.
  $PIP install -r "$SRC/pip-packages.txt" || \
    echo "    (some pip packages failed — check for version pins that don't exist)"
fi

# ---------------------------------------------------------------------------
# 4. cargo
# ---------------------------------------------------------------------------
if [[ -f "$SRC/cargo-packages.txt" ]] && command -v cargo >/dev/null 2>&1; then
  echo "  - installing cargo packages"
  # `cargo install --list` lines look like:  ripgrep v13.0.0:
  # so we take the first field of lines that start with a letter.
  awk '/^[a-zA-Z]/ {print $1}' "$SRC/cargo-packages.txt" | while read -r crate; do
    [[ -n "$crate" ]] && cargo install "$crate" || true
  done
fi

# ---------------------------------------------------------------------------
# 5. npm globals
# ---------------------------------------------------------------------------
if [[ -f "$SRC/npm-packages.txt" ]] && command -v npm >/dev/null 2>&1; then
  echo "  - installing npm globals"
  # `npm list -g` lines look like:  +-- pkg@1.2.3  — strip the tree chars + version.
  awk -F'[ @]' '/[+`\\]-- / {print $3}' "$SRC/npm-packages.txt" | while read -r pkg; do
    [[ -n "$pkg" ]] && sudo npm install -g "$pkg" || true
  done
fi

# ---------------------------------------------------------------------------
# 6. snaps
# ---------------------------------------------------------------------------
if [[ -f "$SRC/snap-packages.txt" ]] && command -v snap >/dev/null 2>&1; then
  echo "  - installing snaps"
  while read -r s; do
    [[ -n "$s" ]] && sudo snap install "$s" || true
  done < "$SRC/snap-packages.txt"
fi

# ---------------------------------------------------------------------------
# 7. flatpaks
# ---------------------------------------------------------------------------
if [[ -f "$SRC/flatpak-packages.txt" ]] && command -v flatpak >/dev/null 2>&1; then
  echo "  - installing flatpaks"
  while read -r f; do
    [[ -n "$f" ]] && flatpak install -y flathub "$f" || true
  done < "$SRC/flatpak-packages.txt"
fi

# ---------------------------------------------------------------------------
# 8. udev rules — then reload so devices pick them up without a reboot
# ---------------------------------------------------------------------------
if [[ -d "$SRC/udev" ]]; then
  echo "  - restoring udev rules"
  sudo cp -r "$SRC"/udev/* /etc/udev/rules.d/ 2>/dev/null || true
  sudo udevadm control --reload-rules && sudo udevadm trigger || true
fi

echo ""
echo "==> Done."
echo "    Review $SRC/MANUAL-TODO.txt for the things not handled automatically"
echo "    (source builds, dotfiles, symlinks, /usr/local, credentials, etc.)."
