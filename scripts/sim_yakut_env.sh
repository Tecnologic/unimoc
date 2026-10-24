# SPDX-License-Identifier: GPL-3.0-or-later
#
# Source this file to point yakut (pycyphal) at the native_sim drive:
#
#   . unimoc/scripts/sim_yakut_env.sh              # SocketCAN vcan0, CAN FD
#   . unimoc/scripts/sim_yakut_env.sh /dev/pts/3   # an slcan pseudo terminal, classic CAN
#
# The simulation prints its pseudo terminals at boot ("uart_1 connected to
# pseudotty: /dev/pts/N"); uart_1 to uart_4 carry slcan, one tool process
# each (uart_0 is the console). Sets CYPHAL_PATH to the standard and the
# UNIMOC DSDL namespaces (yakut compiles them on first use) and the local
# node-ID to 127 unless UAVCAN__NODE__ID is already set.

_unimoc_repo="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
_unimoc_top="$(cd "$_unimoc_repo/.." && pwd)"
export CYPHAL_PATH="$_unimoc_top/modules/lib/public_regulated_data_types:$_unimoc_repo/dsdl"
export UAVCAN__NODE__ID="${UAVCAN__NODE__ID:-127}"

if [[ -n "${1:-}" ]]; then
  # pycyphal runs slcan with classic frames only.
  export UAVCAN__CAN__IFACE="slcan:$1"
  export UAVCAN__CAN__MTU=8
  export UAVCAN__CAN__BITRATE="1000000 1000000"
else
  export UAVCAN__CAN__IFACE="socketcan:vcan0"
  export UAVCAN__CAN__MTU=64
  export UAVCAN__CAN__BITRATE="1000000 4000000"
fi
echo "yakut -> ${UAVCAN__CAN__IFACE} (MTU ${UAVCAN__CAN__MTU}), local node-ID ${UAVCAN__NODE__ID}"
unset _unimoc_repo _unimoc_top
