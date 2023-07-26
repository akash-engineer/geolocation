#!/usr/bin/python

import argparse
import glob
import os
import re
import struct
import sys

def split_yaml(filename):
    magic = "INNODATA1000"
    with open(args.input, 'rb') as f:
        mread = f.read(len(magic))
        if not mread:
            print("end of input file {}".format(filename))
            exit(1)
        if str(mread) != magic:
            print("no magic", mread)
            return None, 0
        yread = f.read(4)
        yaml_length = struct.unpack('>i', yread)[0]
        dread = f.read(yaml_length)
        return mread + yread + dread, len(magic) + 4 + yaml_length
  
def split_raw(args):
    rawdata_size = [
        16,
        6,
        0,
        6,
        2,
        6,
        16,
        6,
        4,
        6,
        0,
        6,
        0,
        6,
        0,
        6,
    ];
    if not args.output:
        output = args.input
    else:
        output = args.output
    filename, file_extension = os.path.splitext(output)
    w_file = None

    yaml, yaml_size = split_yaml(args.input)
    if yaml:
        fn = "{}.yaml".format(filename)
        print("write yaml {} bytes to {}".format(yaml_size, fn))
        with open(fn, 'wb') as y_file:
            y_file.write(yaml)

    with open(args.input, 'rb') as f:
        written_this_file = 0
        skipped = 0;
        total_written = 0
        file_index = 0
        skip_print = False

        if yaml_size:
            f.seek(yaml_size)

        while 1:
            bc = f.read(1)
            if not bc:
                print("end of input file {}".format(args.input))
                break
            b = struct.unpack('B', bc)[0]
            type = b & 0xf
            s = rawdata_size[type]
            # print("{} {} {}".format(type, s, total_written))
            if s - 1 < 0:
                print("cannot handle type {}".format(type))
                exit(-1)
            bs = f.read(s - 1)
            if args.skip and args.skip > skipped:
                skipped += s
                continue
            if skipped and not skip_print:
                skip_print = True
                print("skipped {} bytes".format(skipped))

            if args.total_size and args.total_size < total_written + s:
                print("written {} bytes reachs max size {}".format(total_written,
                                                                   args.total_size))
                break
            total_written += s
            if w_file is None or (args.max_file_size and written_this_file + s >= args.max_file_size):
                if w_file:
                    w_file.close()
                fn = "{}-{}{}".format(filename, file_index, file_extension)
                print("open {} to write".format(fn))
                file_index += 1
                w_file = open(fn, 'wb')
                if yaml:
                    w_file.write(yaml)
                written_this_file = 0
            w_file.write(bc)
            w_file.write(bs)
            written_this_file += s
    if w_file:
        w_file.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", "-i", type=str, default="", help="git repo tag or commit-id")
    parser.add_argument("--output", "-o", type=str, default="", help="output file")
    parser.add_argument("--skip", type=int, default=0, help="skip how many bytes of the head of source file")
    parser.add_argument("--total-size", "-t", type=int, default=0, help="write how many bytes to output files in total")
    parser.add_argument("--max-file-size", "-m", type=int, default=0, help="max size (in byte) of each output file (yaml size is not counted)")
    parser.add_argument("-s", "--show-sample", action="store_true", help="show sample", default=False)
    args = parser.parse_args()
    if args.show_sample:
        sample = '''
- see all command line options:
  {cmd} -h

- split source.dat to multiple files, the max size of each output file is 100000000
  {cmd} -i source.dat -m 100000000

- split source.dat to multiple files, the max size of each output file is 100000000, output file are /tmp/output-xx.dat
  {cmd} -i source.dat -m 100000000 -o /tmp/output-xx.dat

- split source.dat to multiple files, the max size of each output file is 100000000, skip first 1000000 bytes
  {cmd} -i source.dat -m 100000000 --skip 1000000

- split source.dat to multiple files, the max size of each output file is 100000000, skip first 1000000 bytes, write 200000000 at most
  {cmd} -i source.dat -m 100000000 --skip 1000000 -t 200000000
        '''
        print(sample.format(cmd=sys.argv[0]))
        exit(0)

    if not args.input:
        print("error: argument --input/-i is required")
        exit(0)

    split_raw(args)
