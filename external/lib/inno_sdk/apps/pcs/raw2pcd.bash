#! /usr/bin/env bash

if [[ $# -lt 1 || $# -gt 3 ]]; then
    echo "Usage: " $0 "<RAW_FILENAME> [PCD_FRAME_COUNT]"
    exit
fi

FILE=$1

if [[ $# -gt 1 ]]; then
    PCDC=$2
else
    PCDC=10
fi

FILE_NOEXT=${FILE%.*}

./inno_pc_server --file ${FILE} --record-inno-pc-filename ${FILE_NOEXT} --record-inno-pc-size-in-m -1

../example/get_pcd --inno-pc-filename ${FILE_NOEXT}.inno_pc --pcd-frame-count ${PCDC} --pcd-filename ${FILE_NOEXT}.pcd
