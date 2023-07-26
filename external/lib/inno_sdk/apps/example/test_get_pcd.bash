#!/bin/bash 
if [ "$#" -ne 1 ]; then
    echo "Usage: " $0 INNO_PC_FILENAME
    exit
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

set -x
set -e

GET_PCD=./get_pcd

# extract the first 10 frame in inno_pc
${GET_PCD} --inno-pc-filename $1 --frame-start 0 --frame-number 10 --output-filename test.inno_pc
diff $1 test.inno_pc

# split the first 10 frame in inno_pc to 2 inno_pc files
${GET_PCD} --inno-pc-filename $1 --frame-start 0 --frame-number 5 --file-number 2 --output-filename test.inno_pc

# convert inno_pc to inno_pc_xyz
${GET_PCD} --inno-pc-filename $1 --frame-start 0 --frame-number 10 --output-filename test.inno_pc_xyz

# convert inno_pc to pcd
${GET_PCD} --inno-pc-filename $1 --frame-start 0 --frame-number 10 --output-filename test1.pcd
# convert inno_pc_xyz to pcd
${GET_PCD} --inno-pc-filename test.inno_pc_xyz --frame-start 0 --frame-number 100 --output-filename test2.pcd
diff test1.pcd test2.pcd

# convert inno_pc to csv
${GET_PCD} --inno-pc-filename $1 --frame-start 0 --frame-number 2 --output-filename test1.csv
# convert inno_pc_xyz to csv
${GET_PCD} --inno-pc-filename test.inno_pc_xyz --frame-start 0 --frame-number 2 --output-filename test2.csv
diff test1.csv test2.csv

echo -e "${GREEN}PASS${NC}"
