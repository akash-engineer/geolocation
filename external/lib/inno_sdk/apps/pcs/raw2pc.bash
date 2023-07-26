#! /usr/bin/env bash

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: " $0 "RAW_FILENAME"
    exit
fi

FILE=$1
FILE_NOEXT=${FILE%.*}

./inno_pc_server --file ${FILE} --record-inno-pc-filename ${FILE_NOEXT} --record-inno-pc-size-in-m -1
