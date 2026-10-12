#!/usr/bin/env bash
# Configures the Orange Pi's WiFi radio to broadcast its own access point on
# every boot, so the laptop (or anything else) can connect directly to the
# Pi without depending on a phone hotspot or a shared network being
# available - the phone hotspot approach was found unreliable ("hotspot
# doesn't always work").
#
# Uses NetworkManager (already the network manager on this Ubuntu Desktop
# image - confirmed via nmcli being present) rather than hand-rolling
# hostapd/dnsmasq: NetworkManager's own "shared" mode already runs dnsmasq
# and hostapd under the hood, persists as a normal connection profile, and
# autoconnects on boot without a separate systemd unit to maintain.
#
# Run this ONCE, on the Pi, with sudo. Idempotent - safe to re-run.
set -euo pipefail

SSID="${1:-teknofest-robot}"
# No default password on purpose (a real one used to be hardcoded here and
# would have ended up in git history) - pass it as the 2nd argument or via
# the HOTSPOT_PASSWORD environment variable.
PASSWORD="${2:-${HOTSPOT_PASSWORD:-}}"
CON_NAME="teknofest-hotspot"

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0 [ssid] <password>   (or HOTSPOT_PASSWORD=... sudo -E $0)" >&2
  exit 1
fi

if [ -z "$PASSWORD" ]; then
  echo "Hotspot password required: pass it as the 2nd argument or set HOTSPOT_PASSWORD" >&2
  exit 1
fi

if [ ${#PASSWORD} -lt 8 ]; then
  echo "WPA2 password must be at least 8 characters" >&2
  exit 1
fi

# Find the WiFi interface (skip anything already a hotspot from a prior run).
WIFI_IF=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2=="wifi"{print $1; exit}')
if [ -z "$WIFI_IF" ]; then
  echo "No WiFi device found (nmcli device status shows no 'wifi' type device)" >&2
  exit 1
fi
echo "Using WiFi interface: $WIFI_IF"

# Remove a stale profile from a previous run so this is safely re-runnable
# with different SSID/password.
nmcli connection delete "$CON_NAME" 2>/dev/null || true

# ipv4.method shared: NetworkManager runs its own dnsmasq/NAT for this
# connection automatically - no manual dnsmasq.conf/hostapd.conf needed.
# connection.autoconnect + autoconnect-priority high: comes up on boot
# without needing to already have another network to prefer over it.
nmcli connection add \
  type wifi \
  ifname "$WIFI_IF" \
  con-name "$CON_NAME" \
  autoconnect yes \
  connection.autoconnect-priority 100 \
  ssid "$SSID" \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  ipv4.method shared \
  ipv6.method disabled

nmcli connection modify "$CON_NAME" \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "$PASSWORD"

# Bring it up now so this run is verifiable immediately, not just on next boot.
nmcli connection up "$CON_NAME"

echo
echo "Hotspot '$SSID' is up and will autoconnect on every boot."
echo "Connect the laptop to it, then reach the Pi at whatever IP"
echo "'ip addr show $WIFI_IF' reports (NetworkManager's shared mode"
echo "defaults to 10.42.0.1 for the Pi itself, handing out 10.42.0.x"
echo "to clients via its built-in dnsmasq)."
echo
echo "To remove: sudo nmcli connection delete $CON_NAME"
