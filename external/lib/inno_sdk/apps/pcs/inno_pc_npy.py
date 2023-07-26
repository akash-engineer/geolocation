#!/usr/bin/env python
from __future__ import unicode_literals
import argparse
import json
import math
import numpy as np
import os


class InnoPcRecords:
    def __init__(self, file):
        self.filename = file
        with open(file, 'r') as infile:
            # Variable for building our JSON block
            json_block = []
            for line in infile:
                # Add the line to our JSON block
                json_block.append(line)
                # Check whether we closed our JSON block
                if '}' in line:
                    # Do something with the JSON dictionary
                    self.json_dict = json.loads(''.join(json_block))
                    break
            else:
                assert False, "invalid file"
            self.raw = None
            self.xyz = None

        for name in self.json_dict:
            if name == "inno_point_dtype":
                dt = self.get_dtype_(self.json_dict[name])
                setattr(self, name, dt)
            else:
                setattr(self, name, self.json_dict[name])
            print("{}={}".format(name, getattr(self, name)))
        print("")
        self.load_xyz_()
        self.build_frame_idx_()

    def get_dtype_(self, inno_description):
        dtype_array = []
        for i in inno_description.split(','):
            s = i.split('=')
            # old numpy doesn't take unicode
            dtype_array.append((s[0].encode("ascii"),
                                s[1].encode("ascii")))
        dt = np.dtype(dtype_array)
        return dt

    def load_raw_(self):
        with open(self.filename, 'r') as infile:
            infile.seek(self.inno_header_size)
            t = np.fromfile(infile, dtype=self.inno_point_dtype)
            self.points_number = t.shape[0]
            print("loaded {} points".format(self.points_number))
        self.raw = t
        return self.raw

    def load_xyz_(self):
        if self.raw is None:
            self.load_raw_()
        if self.inno_pc_records_version == "1.0":
            rad_in_unit = math.pi / self.inno_angle_unit_per_PI_Rad
            meter_in_unit = 1.0 / self.inno_distance_unit_per_Meter

            # angles
            self.h_angle = self.raw['h_angle_unit'] * rad_in_unit
            self.v_angle = self.raw['v_angle_unit'] * rad_in_unit
            sin_h = np.sin(self.h_angle)
            cos_h = np.cos(self.h_angle)
            sin_v = np.sin(self.v_angle)
            cos_v = np.cos(self.v_angle)
            print("h-angles: {}".format(self.h_angle))
            print("v-angles: {}".format(self.v_angle))

            # radius
            # self.raw['radius_unit'] has low 16-bit
            radius = self.raw['radius_unit'].astype('<u4')
            radius[(self.raw['refl'] & (1 << 15)) != 0] |= (1 << 16)
            self.radius = radius * meter_in_unit;
            print("radius: {}".format(self.radius))

            # reflectance
            self.refl = self.raw['refl'] & 0x1ff
            print("reflectance: {}".format(self.refl))

            # x, y, z
            t = self.radius * cos_v;
            self.x = self.radius * sin_v;
            self.y = t * sin_h;
            self.z = t * cos_h;
            print("x: {}".format(self.x))
            print("y: {}".format(self.y))
            print("z: {}".format(self.z))

            # frame_id need to add base
            self.frame_id = self.raw['frame_id'] + self.inno_frame_id_base
            print("frame_id: {}".format(self.frame_id))

            # timestamp need to add base
            self.timestamp_10us = self.raw['timestamp_10us'] + self.inno_timestamp_base_10us
            print("timestamp_10us: {}".format(self.timestamp_10us))
        else:
            assert False, "version {} not supported".format(self.inno_pc_records_version)

    def build_frame_idx_(self):
        self.frame_idx = []
        self.real_frame_idx = {}
        invalid_frame_id = -1000
        current_fi = invalid_frame_id
        point_count = 0
        for i in xrange(self.points_number):
            this_frame = self.frame_id[i]
            if this_frame == current_fi:
                point_count += 1
            else:
                # new frame
                self.real_frame_idx[this_frame] = len(self.frame_idx)
                if current_fi != invalid_frame_id:
                    self.frame_idx.append((i - point_count, point_count, current_fi))
                point_count = 1
                current_fi = this_frame
        else:
            self.frame_idx.append((self.points_number - point_count, point_count, current_fi))

    def get_frames_count(self):
        return len(self.frame_idx)

    def write_pcd(self, pcd_file, frame_id):
        header = "# .PCD v.7 - Point Cloud Data file format\n" + \
                 "FIELDS x y z i\n" + \
                 "SIZE 4 4 4 4\n" + \
                 "TYPE F F F F\n" + \
                 "COUNT 1 1 1 1\n" + \
                 "WIDTH {}\n" + \
                 "HEIGHT 1\n" + \
                 "VIEWPOINT 0 0 0 1 0 0 0\n" + \
                 "POINTS {}\n" + \
                 "DATA ascii\n"

        if frame_id >= self.get_frames_count():
            print("frame_id cannot be bigger than {}".format(len(self.frame_idx)))
            return

        info = self.frame_idx[frame_id]
        start_position = info[0]
        n_points = info[1]
        real_frame_id = info[2]
        target_frame = self.frame_id[start_position]
        assert real_frame_id == target_frame, "id mismatch {} vs {}".format(real_frame_id, target_frame)
        fn = os.path.splitext(pcd_file)
        fns = fn[0] + ".{}.{}".format(frame_id, real_frame_id) + fn[1]
        if len(fn[1]) == 0:
            fns += ".pcd"

        with open(fns, 'w') as out:
            real_header = header.format(n_points, n_points)
            out.write(real_header)
            print("write {} points from frame {} to {}".format(n_points, real_frame_id, fns))

            for m in xrange(n_points):
                k = m + start_position
                out.write("{} {} {} {}\n".format(self.x[k], self.y[k],
                                                 self.z[k],
                                                 self.refl[k]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="inno_pc_npy processor")
    parser.add_argument("-i", "--input", type=str, required=True, help='input inno_pc_npy file')
    parser.add_argument("-o", "--output-pcd", type=str, default="", help='output pcd file')
    parser.add_argument("-f", "--frame-idx", nargs='+', type=int, default=[0],
                        help='nth frame to select, -1 means write all')
    args = parser.parse_args()

    records = InnoPcRecords(args.input)

    if (args.output_pcd):
        if -1 in args.frame_idx:
            for i in xrange(records.get_frames_count()):
                records.write_pcd(args.output_pcd, i)
        else:
            for i in args.frame_idx:
                records.write_pcd(args.output_pcd, i)
