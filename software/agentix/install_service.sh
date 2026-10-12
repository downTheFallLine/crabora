#!/usr/bin/env bash
# Install agentix as a systemd service, started at boot.
#
# Run it on the ROBOT, not the Mac:
#     cd ~/crabora/software/agentix && ./install_service.sh
#
# Two things are set up:
#   1. the unit itself, with paths resolved for this machine
#   2. /etc/crabora/agentix.env -- root-owned, 0600, where the API key
#      lives. It is outside the repo on purpose: a key in a working tree
#      is one `git add -A` away from being published.
#
# Safe to re-run. It rewrites the unit and leaves an existing env file
# alone unless you ask for the credentials to be set again.

set -euo pipefail

UNIT_NAME="crabora-agentix.service"
ENV_DIR="/etc/crabora"
ENV_FILE="${ENV_DIR}/agentix.env"
APPDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${APPDIR}/${UNIT_NAME}"

die() { echo "error: $*" >&2; exit 1; }

[[ -f "${TEMPLATE}" ]] || die "cannot find ${TEMPLATE}"
[[ -f "${APPDIR}/app.py" ]] || die "no app.py in ${APPDIR}"
command -v systemctl >/dev/null || die "no systemd on this machine"

# Run as the invoking user, not root -- the service should own the serial
# ports as that user, and sudo -E would bake root's environment in.
RUN_USER="${SUDO_USER:-$(id -un)}"
[[ "${RUN_USER}" != "root" ]] || die "run as the normal user (it will sudo as needed), not as root"

# --- find the interpreter -------------------------------------------
# Prefer a venv: the Pi is PEP 668 externally-managed, so the deps almost
# certainly live in one rather than in the system python.
PYTHON=""
for cand in "${APPDIR}/../../.venv/bin/python3" \
            "${APPDIR}/../.venv/bin/python3" \
            "${APPDIR}/.venv/bin/python3"; do
    [[ -x "${cand}" ]] && { PYTHON="$(cd "$(dirname "${cand}")" && pwd)/$(basename "${cand}")"; break; }
done
[[ -n "${PYTHON}" ]] || PYTHON="$(command -v python3 || true)"
[[ -n "${PYTHON}" ]] || die "no python3 found"

"${PYTHON}" -c "import flask" 2>/dev/null \
    || die "flask is not installed for ${PYTHON} -- activate the venv and pip install flask, then re-run"

echo "  user:   ${RUN_USER}"
echo "  app:    ${APPDIR}"
echo "  python: ${PYTHON}"

# --- serial access ---------------------------------------------------
if ! id -nG "${RUN_USER}" | tr ' ' '\n' | grep -qx dialout; then
    echo
    echo "  ${RUN_USER} is not in the dialout group, so it cannot open /dev/ttyUSB*."
    read -r -p "  Add it? [y/N] " yn
    if [[ "${yn}" == "y" || "${yn}" == "Y" ]]; then
        sudo usermod -aG dialout "${RUN_USER}"
        echo "  added -- this takes effect for the service immediately,"
        echo "  but your own shell needs a re-login to see it."
    else
        echo "  skipped: the UI will run but the bus will report unavailable."
    fi
fi

# --- credentials -----------------------------------------------------
write_env=yes
if [[ -f "${ENV_FILE}" ]]; then
    echo
    read -r -p "  ${ENV_FILE} exists. Replace the credentials? [y/N] " yn
    [[ "${yn}" == "y" || "${yn}" == "Y" ]] || write_env=no
fi

if [[ "${write_env}" == "yes" ]]; then
    echo
    echo "  API key for chat. Leave blank to skip -- the UI and all tools"
    echo "  work without one; only the chat box is disabled."
    # -s: never echoed, and this never reaches the shell history or the repo.
    read -r -s -p "  ANTHROPIC_API_KEY: " api_key; echo
    workspace=""
    if [[ -n "${api_key}" ]]; then
        echo
        echo "  Workspace id, only for an ORG-level key (starts wrkspc_)."
        echo "  A workspace-scoped key carries its own scope -- leave blank."
        read -r -p "  ANTHROPIC_WORKSPACE_ID: " workspace
    fi

    sudo install -d -m 0755 "${ENV_DIR}"
    # Written 0600 root-owned via a temp file so the key is never briefly
    # world-readable, and never lands in the repo working tree.
    tmp="$(mktemp)"
    chmod 600 "${tmp}"
    {
        echo "# agentix credentials. Root-owned, 0600. NOT in the repo."
        echo "# Regenerate with software/agentix/install_service.sh"
        [[ -n "${api_key}"   ]] && echo "ANTHROPIC_API_KEY=${api_key}"
        [[ -n "${workspace}" ]] && echo "ANTHROPIC_WORKSPACE_ID=${workspace}"
    } > "${tmp}"
    sudo install -o root -g root -m 0600 "${tmp}" "${ENV_FILE}"
    rm -f "${tmp}"
    echo "  wrote ${ENV_FILE} (root:root 0600)"
fi

# --- the unit --------------------------------------------------------
tmp_unit="$(mktemp)"
sed -e "s|__USER__|${RUN_USER}|g" \
    -e "s|__APPDIR__|${APPDIR}|g" \
    -e "s|__PYTHON__|${PYTHON}|g" \
    "${TEMPLATE}" > "${tmp_unit}"
sudo install -o root -g root -m 0644 "${tmp_unit}" "/etc/systemd/system/${UNIT_NAME}"
rm -f "${tmp_unit}"

sudo systemctl daemon-reload
sudo systemctl enable "${UNIT_NAME}" >/dev/null
sudo systemctl restart "${UNIT_NAME}"

echo
sleep 2
if systemctl is-active --quiet "${UNIT_NAME}"; then
    ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
    echo "  running, and will start at boot."
    echo "  http://${ip:-<this-pi>}:5000/"
else
    echo "  FAILED to start. Last lines:"
    sudo journalctl -u "${UNIT_NAME}" -n 25 --no-pager
    exit 1
fi

cat <<EOF

  logs      sudo journalctl -u ${UNIT_NAME} -f
  restart   sudo systemctl restart ${UNIT_NAME}
  stop      sudo systemctl stop ${UNIT_NAME}
  disable   sudo systemctl disable --now ${UNIT_NAME}

  Stop the service before running a gait program by hand -- both want the
  same serial ports, and the loser gets a confusing permission error.
EOF
