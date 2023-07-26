#!/usr/bin/env bash
IP=$1
MYDIR="$(dirname "$(realpath "$0")")"
CMD="${MYDIR}/../../innovusion_lidar_util ${IP}"

if [[ $# -lt 1 || $# -gt 1 ]]; then
    echo "Usage: " $0 " <LIDAR-IP-ADDRESS>"
    exit
fi

${CMD} upload_internal_file PCS_CFG $MYDIR/PCS_CFG
${CMD} upload_internal_file PCS_ENV $MYDIR/PCS_ENV
${CMD} set_config manufacture internal_server 1
