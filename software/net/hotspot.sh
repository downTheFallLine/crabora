#!/usr/bin/env bash
# Make the robot its own WiFi access point, so a laptop can connect
# directly to it with no infrastructure in the way.
#
# Run it ON THE ROBOT:
#     cd ~/crabora/software/net && ./hotspot.sh
#
#     ./hotspot.sh --status    what is it doing right now
#     ./hotspot.sh --off       stop being an AP, go back to the house WiFi
#     ./hotspot.sh --now       switch to AP immediately, not just on fallback
#
# WHAT IT SETS UP, and why this shape:
#
# The Pi has ONE radio, and concurrent AP+client on brcmfmac is flaky --
# drops, wedged interfaces, poor throughput. So it is one or the other,
# and the interesting question is which one, when.
#
# The answer here is FALLBACK, not a switch: the house WiFi keeps a high
# autoconnect priority and the AP profile gets a low one. An AP always
# "succeeds", so it must sort last or it would win every time. Result:
# at home the robot joins the house network as usual, and out in the
# garden -- where there is no house network -- it becomes its own AP and
# you connect straight to it. Nothing to remember, nothing to toggle.
#
# WHILE IT IS AN AP THE ROBOT HAS NO INTERNET. No git pull, no apt. That
# is not a bug in the script, it is the one radio.
#
# NOT TESTED ON HARDWARE. It was written on a Mac with no Pi attached.
# Read it before running it, especially the NetworkManager install, which
# can cut the SSH session you are typing into.

set -euo pipefail

AP_NAME="crabora-ap"
AP_SSID="crabora"
IFACE="${IFACE:-wlan0}"
AP_PRIORITY=-50          # below anything else, so it is the last resort
HOME_PRIORITY=50

say()  { printf '  %s\n' "$*"; }
die()  { printf 'error: %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------
# status
# ---------------------------------------------------------------
show_status() {
    command -v nmcli >/dev/null || die "NetworkManager (nmcli) is not installed"
    local active mode ip
    active="$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null \
              | grep ":${IFACE}\$" | cut -d: -f1 || true)"
    mode="$(nmcli -t -f GENERAL.STATE device show "${IFACE}" 2>/dev/null \
            | cut -d: -f2 || echo unknown)"
    ip="$(ip -4 addr show "${IFACE}" 2>/dev/null \
          | awk '/inet /{print $2}' | cut -d/ -f1 || true)"

    say "interface : ${IFACE} (${mode})"
    say "profile   : ${active:-none active}"
    say "address   : ${ip:-none}"
    if [[ "${active}" == "${AP_NAME}" ]]; then
        say ""
        say "It is its own access point. Join SSID '${AP_SSID}' and open:"
        say "    http://${ip:-10.42.0.1}:5000/"
        say ""
        say "No internet on the robot while this is the case."
    elif [[ -n "${active}" ]]; then
        say ""
        say "On the house network. The AP is configured as a fallback and"
        say "will come up on its own when this network is out of range."
    fi
    echo
    say "profiles, in the order NetworkManager will try them:"
    nmcli -t -f NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY con show \
        | awk -F: '$2 ~ /wireless/ {printf "    %-24s autoconnect=%-5s priority=%s\n", $1, $3, $4}'
}

# ---------------------------------------------------------------
# argument handling
# ---------------------------------------------------------------
MODE="setup"
case "${1:-}" in
    --status) MODE="status" ;;
    --off)    MODE="off" ;;
    --now)    MODE="now" ;;
    "")       MODE="setup" ;;
    *)        die "unknown option: $1 (try --status, --off, --now)" ;;
esac

if [[ "${MODE}" == "status" ]]; then show_status; exit 0; fi

[[ "$(id -un)" != "root" ]] || die "run as the normal user; it sudos as needed"

# ---------------------------------------------------------------
# NetworkManager has to be the one driving wlan0
# ---------------------------------------------------------------
if ! command -v nmcli >/dev/null || ! systemctl is-active --quiet NetworkManager; then
    cat <<'WARN'

  NetworkManager is not running, so this machine is probably on netplan
  with systemd-networkd -- the Ubuntu Server default.

  Installing NetworkManager and handing it wlan0 WILL BRIEFLY DROP THE
  NETWORK, and over SSH that means dropping the session you are typing
  into. If it does not come back, the robot needs a keyboard and monitor,
  or ethernet, to recover.

  Do this part with ethernet plugged in, or sitting in front of it.

WARN
    read -r -p "  Install NetworkManager and continue? [y/N] " yn
    [[ "${yn}" == "y" || "${yn}" == "Y" ]] || { say "Stopped. Nothing changed."; exit 1; }

    say "installing..."
    sudo apt-get update -qq
    sudo apt-get install -y network-manager
    # Hand wifi to NM; netplan keeps ethernet. Written as a separate file
    # so the distribution's own netplan config is left alone.
    sudo install -d -m 0755 /etc/netplan
    printf 'network:\n  version: 2\n  wifis:\n    renderer: NetworkManager\n' \
        | sudo tee /etc/netplan/99-crabora-nm.yaml >/dev/null
    sudo chmod 600 /etc/netplan/99-crabora-nm.yaml
    sudo netplan apply || true
    sudo systemctl enable --now NetworkManager
    sleep 3
    command -v nmcli >/dev/null || die "nmcli still missing after install"
fi

nmcli -t -f DEVICE device status | grep -qx "${IFACE}" \
    || die "no interface called ${IFACE} (set IFACE=... if it is named differently)"

# ---------------------------------------------------------------
# off
# ---------------------------------------------------------------
if [[ "${MODE}" == "off" ]]; then
    if nmcli -t -f NAME con show | grep -qx "${AP_NAME}"; then
        sudo nmcli con modify "${AP_NAME}" connection.autoconnect no
        sudo nmcli con down "${AP_NAME}" 2>/dev/null || true
        say "AP disabled. The profile is kept -- re-enable with ./hotspot.sh"
    else
        say "no ${AP_NAME} profile to disable"
    fi
    echo; show_status; exit 0
fi

# ---------------------------------------------------------------
# the AP profile
# ---------------------------------------------------------------
if nmcli -t -f NAME con show | grep -qx "${AP_NAME}"; then
    say "updating the existing ${AP_NAME} profile"
    read -r -p "  Set a new passphrase? [y/N] " yn
    NEW_PSK=""
    if [[ "${yn}" == "y" || "${yn}" == "Y" ]]; then
        read -r -s -p "  passphrase (8+ chars): " NEW_PSK; echo
        [[ ${#NEW_PSK} -ge 8 ]] || die "WPA2 needs at least 8 characters"
    fi
else
    say "creating ${AP_NAME}"
    # read -s: never echoed, and never reaches the shell history.
    read -r -s -p "  passphrase for SSID '${AP_SSID}' (8+ chars): " NEW_PSK; echo
    # Validate BEFORE creating anything, or a short passphrase leaves a
    # half-built profile behind for the next run to trip over.
    [[ -n "${NEW_PSK}" ]] || die "a passphrase is required"
    [[ ${#NEW_PSK} -ge 8 ]] || die "WPA2 needs at least 8 characters"
    sudo nmcli con add type wifi ifname "${IFACE}" con-name "${AP_NAME}" \
        ssid "${AP_SSID}" >/dev/null
fi

sudo nmcli con modify "${AP_NAME}" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    802-11-wireless.ssid "${AP_SSID}" \
    ipv4.method shared \
    ipv6.method ignore \
    connection.autoconnect yes \
    connection.autoconnect-priority "${AP_PRIORITY}"

# wpa-psk, not open. Anyone on this network reaches the agentix UI with
# no authentication at all, and ARM is the only thing between them and
# the servos.
sudo nmcli con modify "${AP_NAME}" \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.proto rsn \
    802-11-wireless-security.pairwise ccmp \
    802-11-wireless-security.group ccmp
[[ -n "${NEW_PSK}" ]] && sudo nmcli con modify "${AP_NAME}" \
    802-11-wireless-security.psk "${NEW_PSK}"
NEW_PSK=""

# ---------------------------------------------------------------
# make the house networks outrank it
# ---------------------------------------------------------------
# Without this the AP wins at boot, because an AP always comes up and an
# infrastructure profile only comes up if that network is in range.
while IFS=: read -r name type; do
    [[ "${type}" == *wireless* ]] || continue
    [[ "${name}" == "${AP_NAME}" ]] && continue
    sudo nmcli con modify "${name}" \
        connection.autoconnect yes \
        connection.autoconnect-priority "${HOME_PRIORITY}" 2>/dev/null || true
done < <(nmcli -t -f NAME,TYPE con show)

echo
say "done. The robot will join a known network when one is in range, and"
say "fall back to being '${AP_SSID}' when none is."
echo

if [[ "${MODE}" == "now" ]]; then
    cat <<WARN
  Switching to AP mode NOW. If you are connected over WiFi, this will
  drop your SSH session -- the robot is leaving the network you are both
  on. Reconnect by joining '${AP_SSID}' and using 10.42.0.1.

WARN
    read -r -p "  Continue? [y/N] " yn
    if [[ "${yn}" == "y" || "${yn}" == "Y" ]]; then
        # Backgrounded with a delay so this script can exit and report
        # before the link goes down under it.
        sudo sh -c "sleep 2; nmcli con up '${AP_NAME}'" &
        say "switching in 2 seconds..."
        exit 0
    fi
    say "left as a fallback only."
fi

echo
show_status
