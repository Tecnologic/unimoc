#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Starts N simulated drives (native_sim) on one SocketCAN bus, for tools like
# cypilot and multi-node tests (docs/gui_plan.md, section 6):
#
#   unimoc/scripts/sim_cluster.sh                   # 4 drives on vcan0
#   unimoc/scripts/sim_cluster.sh -n 2 -i vcan1     # 2 drives on vcan1
#   unimoc/scripts/sim_cluster.sh --setup-vcan      # create vcan0 first (sudo)
#
# Drive k (0 .. N-1) gets node-ID <first-id> + k (10, 11, ...), its own flash
# file and unique-ID, and cart wheel k mod 4 (front left, front right, rear
# left, rear right; unimoc.cart.WheelCommand.WHEEL_*). The flash files stay in
# the state directory, so settings stored on a drive survive the next start.
# Each drive also opens its slcan pseudo terminals; the log of drive k names
# them. Ctrl+C (or SIGTERM) stops all drives.
#
# The executable is --exe, $UNIMOC_SIM_EXE, or a west build of unimoc/app for
# native_sim/native/64 in build/sim or unimoc/app/build/sim. On Windows run it
# in WSL2 with the tools that use the bus.
set -euo pipefail

usage() {
  sed -n '4,20p' "$0" | sed 's/^# \{0,1\}//'
  cat <<EOF

Options:
  -n, --drives N       number of drives (default 4)
  -i, --interface IF   SocketCAN interface (default vcan0)
  -f, --first-id ID    node-ID of the first drive (default 10)
  -d, --dir DIR        flash files and logs (default \${XDG_STATE_HOME:-~/.local/state}/unimoc-cluster)
  -e, --exe FILE       native_sim executable
      --setup-vcan     create the interface (modprobe vcan, ip link, MTU 72) with sudo
  -h, --help           this text
EOF
}

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
drives=4
interface=vcan0
first_id=10
dir="${XDG_STATE_HOME:-$HOME/.local/state}/unimoc-cluster"
exe="${UNIMOC_SIM_EXE:-}"
setup_vcan=false

while [[ $# -gt 0 ]]; do
  case "$1" in
  -n | --drives) drives="$2"; shift 2 ;;
  -i | --interface) interface="$2"; shift 2 ;;
  -f | --first-id) first_id="$2"; shift 2 ;;
  -d | --dir) dir="$2"; shift 2 ;;
  -e | --exe) exe="$2"; shift 2 ;;
  --setup-vcan) setup_vcan=true; shift ;;
  -h | --help) usage; exit 0 ;;
  *) echo "sim_cluster.sh: unknown option $1 (see --help)" >&2; exit 2 ;;
  esac
done

if ! [[ "$drives" =~ ^[0-9]+$ ]] || ((drives < 1)); then
  echo "sim_cluster.sh: --drives takes a positive number" >&2
  exit 2
fi
if ! [[ "$first_id" =~ ^[0-9]+$ ]] || ((first_id < 1 || first_id + drives - 1 > 127)); then
  echo "sim_cluster.sh: the node-IDs $first_id .. $((first_id + drives - 1)) leave 1 .. 127" >&2
  exit 2
fi

if [[ -z "$exe" ]]; then
  for candidate in "$repo/../build/sim/zephyr/zephyr.exe" "$repo/app/build/sim/zephyr/zephyr.exe"; do
    if [[ -x "$candidate" ]]; then
      exe="$candidate"
      break
    fi
  done
fi
if [[ -z "$exe" || ! -x "$exe" ]]; then
  echo "sim_cluster.sh: no native_sim executable; build one with" >&2
  echo "  west build -b native_sim/native/64 unimoc/app -d build/sim" >&2
  echo "or pass --exe (the release asset unimoc-native_sim-*.tar.gz holds one)" >&2
  exit 1
fi

if [[ ! -e "/sys/class/net/$interface" ]]; then
  if $setup_vcan; then
    sudo modprobe vcan
    sudo ip link add dev "$interface" type vcan
    sudo ip link set "$interface" mtu 72 up  # CAN FD
  else
    echo "sim_cluster.sh: no interface $interface; create it with --setup-vcan or" >&2
    echo "  sudo modprobe vcan && sudo ip link add dev $interface type vcan && sudo ip link set $interface mtu 72 up" >&2
    exit 1
  fi
fi
if [[ "$(cat "/sys/class/net/$interface/mtu")" != 72 ]]; then
  echo "sim_cluster.sh: $interface is not CAN FD (MTU $(cat "/sys/class/net/$interface/mtu"), want 72):" >&2
  echo "  sudo ip link set $interface mtu 72" >&2
  exit 1
fi

mkdir -p "$dir"
pids=()
stop() {
  trap - INT TERM EXIT
  if ((${#pids[@]})); then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
  echo "sim_cluster.sh: stopped"
}
trap 'stop; exit 0' INT TERM
trap stop EXIT

wheels=(front_left front_right rear_left rear_right)
echo "sim_cluster.sh: $drives drive(s) on $interface, executable $exe"
for ((k = 0; k < drives; k++)); do
  node_id=$((first_id + k))
  wheel=$((k % 4))
  log="$dir/drive$node_id.log"
  "$exe" -flash="$dir/drive$node_id.flash" --can-if="$interface" --node-id="$node_id" \
    --device_id="$node_id" --cart-wheel="$wheel" >"$log" 2>&1 &
  pids+=($!)
  echo "  node $node_id: wheel $wheel (${wheels[$wheel]}), log $log"
done

# A drive that ends on its own (a bad option, a lost interface) stops the cluster.
wait -n "${pids[@]}" || true
echo "sim_cluster.sh: a drive ended; see the logs in $dir" >&2
exit 1
