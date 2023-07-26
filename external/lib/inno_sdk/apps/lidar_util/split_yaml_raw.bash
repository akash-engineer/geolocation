#!/usr/bin/env bash

# This script splits the input raw file to two files.
# One file contains the geo-yaml data and the other file
# contains only the raw data.

FILE=$1

if [ $# -lt 1 ] || [ $1 == "-h" ] ; then
    echo "usage: $0 <FILENAME>"
    exit 1
fi
if [ ! -f "$FILE" ] ; then
    echo "$FILE does not exist."
    exit 1
fi

echo Split source file to $FILE.dat and $FILE.yaml.

# file start with "INNODATA1000"
MAGIC_SIZE=12
MAGIC=`head -c $MAGIC_SIZE $FILE`
GOOD_MAGIC="INNODATA1000"

if [ "$MAGIC" != $GOOD_MAGIC ]; then
    echo Not a valid raw-raw file to split. File must start with $GOOD_MAGIC
    exit 1
fi

# then 12th-15th bytes contain the 4-byte YAML_LENGTH in big endian
YAML_LENGTH_SIZE=4
HEADER_SIZE=$(($MAGIC_SIZE + $YAML_LENGTH_SIZE))
YAML_LENGTH=`head -c $HEADER_SIZE $FILE | tail -c $YAML_LENGTH_SIZE |od  --endian big -t u4 -An|tr -d ' '`

# then data
DATA_OFFSET=$(($YAML_LENGTH + $HEADER_SIZE))
echo Data offset is $DATA_OFFSET

filesize=$(wc -c <$1)
tail -c $(($filesize-$DATA_OFFSET)) $FILE > $FILE.dat
head -c $(($DATA_OFFSET)) $FILE | tail -c $(($DATA_OFFSET-$HEADER_SIZE)) | sed '$ s/\x00*$//' > $FILE.yaml
echo Done.
