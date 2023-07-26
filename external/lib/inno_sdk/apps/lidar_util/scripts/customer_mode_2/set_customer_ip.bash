#!/usr/bin/env bash
IP=$1
MYDIR="$(dirname "$(realpath "$0")")"
CMD="${MYDIR}/../../innovusion_lidar_util ${IP}"

if [[ $# -lt 1 || $# -gt 1 ]]; then
    echo "Usage: " $0 " <LIDAR-IP-ADDRESS>"
    exit
fi

# ${CMD} set_network 10.42.0.91 255.255.255.0
echo "To change IP, please execute: ${CMD} set_network <NEW-IP> <NEW-NETWORK-MASK>"
