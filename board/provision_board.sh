#!/bin/bash
# =============================================================================
# provision_board.sh
#
# One-shot provisioning script for a RISC-V board in the archfuzz-riscv lab.
#
# Creates the standard "archfuzz" user, grants it serial-port access, builds
# runner.c (expected alongside this script) into that user's home directory,
# and installs/enables a systemd service so `runner` starts automatically on
# every future boot, listening on the given UART device. After this runs
# once, the board never needs manual console interaction to start `runner`
# again - it comes up on its own every boot.
#
# Run once per board, as root, with the UART device wired to the FPGA for
# that board as the argument. Safe to re-run: an existing user/service is
# updated in place rather than duplicated.
#
# Usage:
#   sudo ./provision_board.sh /dev/ttyS1 [path/to/id_ed25519.pub]
#
# The optional second argument installs an SSH public key for the archfuzz
# user so it can be logged into remotely once the board has network. Without
# it, the account is created with no password and no key - only usable via
# the systemd service - until you run `sudo passwd archfuzz` or add a key
# yourself.
# =============================================================================

set -euo pipefail

USERNAME="archfuzz"
UART_DEV="${1:-}"
PUBKEY_PATH="${2:-}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOME_DIR="/home/${USERNAME}"
RUNNER_SRC="${SCRIPT_DIR}/runner.c"
RUNNER_BIN="${HOME_DIR}/runner"
SERVICE_FILE="/etc/systemd/system/runner.service"

if [ -z "$UART_DEV" ]; then
    echo "Usage: sudo $0 <uart-device> [path/to/pubkey]" >&2
    echo "  e.g. sudo $0 /dev/ttyS1" >&2
    exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "Must be run as root: sudo $0 $UART_DEV ${PUBKEY_PATH}" >&2
    exit 1
fi

# --- User setup --------------------------------------------------------------
if id "$USERNAME" >/dev/null 2>&1; then
    echo "[*] User '$USERNAME' already exists, skipping creation."
else
    echo "[*] Creating user '$USERNAME'..."
    adduser --disabled-password --gecos "" "$USERNAME"
fi

echo "[*] Adding '$USERNAME' to dialout and sudo groups..."
usermod -aG dialout "$USERNAME"
usermod -aG sudo "$USERNAME"

if [ -n "$PUBKEY_PATH" ]; then
    if [ -f "$PUBKEY_PATH" ]; then
        echo "[*] Installing SSH public key for '$USERNAME'..."
        install -d -m 700 -o "$USERNAME" -g "$USERNAME" "${HOME_DIR}/.ssh"
        cat "$PUBKEY_PATH" >> "${HOME_DIR}/.ssh/authorized_keys"
        sort -u -o "${HOME_DIR}/.ssh/authorized_keys" "${HOME_DIR}/.ssh/authorized_keys"
        chmod 600 "${HOME_DIR}/.ssh/authorized_keys"
        chown "${USERNAME}:${USERNAME}" "${HOME_DIR}/.ssh/authorized_keys"
    else
        echo "[!] Public key file '$PUBKEY_PATH' not found - skipping key install." >&2
    fi
else
    echo "[*] No public key given - '$USERNAME' has no password and no SSH key yet."
    echo "    Run 'sudo passwd $USERNAME' or add a key later if you need to log in directly."
fi

# --- Build runner --------------------------------------------------------------
if [ -f "$RUNNER_SRC" ]; then
    if command -v gcc >/dev/null 2>&1; then
        echo "[*] Compiling runner.c..."
        gcc -O2 -Wall -Wextra -std=gnu11 -o "$RUNNER_BIN" "$RUNNER_SRC"
        chown "${USERNAME}:${USERNAME}" "$RUNNER_BIN"
        chmod 755 "$RUNNER_BIN"
    else
        echo "[!] gcc not found - cannot build runner.c." >&2
        echo "    Install a compiler (or cross-compile elsewhere and copy the" >&2
        echo "    binary to ${RUNNER_BIN}), then rerun this script." >&2
        exit 1
    fi
elif [ -x "$RUNNER_BIN" ]; then
    echo "[*] runner.c not found next to this script, but ${RUNNER_BIN} already exists - using it."
else
    echo "[!] No runner.c next to this script, and no existing ${RUNNER_BIN} binary." >&2
    echo "    Place runner.c alongside $0 (or install a prebuilt binary at" >&2
    echo "    ${RUNNER_BIN}) and rerun." >&2
    exit 1
fi

# --- systemd service -----------------------------------------------------------
echo "[*] Installing systemd service (UART: ${UART_DEV})..."
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=RISC-V differential fuzzing runner
After=multi-user.target

[Service]
ExecStart=${RUNNER_BIN} ${UART_DEV}
Restart=always
RestartSec=2
User=${USERNAME}

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now runner.service

echo
echo "[*] Done. Service status:"
systemctl status runner.service --no-pager || true
