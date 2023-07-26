# import dpkt
import argparse
import struct
import time
import math
#from scapy.all import rdpcap

import os

import numpy as np
import pandas as pd
import inno_falcon_b_parse as falcon
import scapy

class pcd_saver(object):
    def __init__(self):
        pass

    def save(self,nparray,file_path):
        header = self._generate_header()
        df     = pd.DataFrame(nparray)
        self._save_to_pcd(header,df,file_path)
        return len(df)

    def _generate_header(self):
        header = "VERSION 0.7\n\
FIELDS x y z intensity timestamp scanidchannel\n\
SIZE 4 4 4 4 4 4\n\
TYPE F F F F F F\n\
COUNT 1 1 1 1 1 1\n\
WIDTH XXXX\n\
HEIGHT 1\n\
VIEWPOINT 0 0 0 1 0 0 0\n\
POINTS XXXX\n\
DATA ascii"
        return header

    def _save_to_pcd(self, header, df, pcd_file):
        length = df.shape[0]
        np.savetxt(pcd_file, df.values, fmt='%f')
        with open(pcd_file, 'r+') as f:
            content = f.read()
            f.seek(0, 0)
            new_header = header.replace("XXXX", str(length))
            f.write(new_header)
            f.write("\n")
            f.write(content)

INNO_CFRAME_NONE = 0
INNO_CFRAME_POINT = 1
INNO_CFRAME_CPOINT = 2
INNO_CFRAME_BBOX = 3
INNO_CFRAME_TEXT = 4
INNO_CFRAME_ALARM = 5
INNO_CFRAME_MAX = 6
cpoint_angel_unit_c = math.pi / 8192.0
cframe_geo_co_unit_c = 0.00000000174532927777
cframe_geo_co_unit_degree_c = 1.0/10000000
INNO_DATA_PACKAGE = 1
INNO_MESSAGE_PACKAGE = 2
INNO_STATUS_PACKAGE = 3

class inno_point(object):
    def __init__(self, raw_buf=None):
        self.buf_size = 24
        self.is_initialized = False

        if raw_buf is None:
            self.x = 0
            self.y = 0
            self.z = 0
            self.radius = 0
            self.ts_100us = 0
            self.ref = 0
            self.flags = 0
        else:
            self.decode(raw_buf)

    def decode(self, raw_buf):
        self.x, self.y, self.z,\
            self.radius, self.ts_100us,\
            self.ref, self.flags = struct.unpack("4f2HB", raw_buf[0:21])
        # scan_id,scan_idx,reserved = struct.unpack("3I",point_buff[21:33])

        self.is_initialized = True

    def print_info(self):
        print('Point Info:')
        print('    x:%f,y:%f,z:%f,radius:%u,ts_100us:%d,ref:%d,flags:%u' % (
            self.x, self.y, self.z, self.radius, self.ts_100us, self.ref, self.flags))


class inno_cpoint(object):
    def __init__(self, raw_buf=None):
        #
        self.buf_size = 12
        self.is_initialized = False
        self.radius = 0
        self.h_angle = 0
        self.v_angle = 0
        self.ts_100us = 0
        self.ts = 0
        self.scan_id = 0
        self.flags = 0
        self.scan_idx = 0
        self.ref = 0
        if raw_buf is None:
            pass
        else:
            self.decode(raw_buf)

    def decode(self, raw_buf):
        self.radius = struct.unpack('H', raw_buf[0:2])[0]/100.0
        self.h_angle = (raw_buf[2]) | ((raw_buf[3] & 0b00011111) << 8)
        self.v_angle = ((raw_buf[5] & 0b00000001) << 11) | (
            raw_buf[4] << 3) | ((raw_buf[3] >> 5) & 0b00000111)
        if (self.h_angle > 2047 * 2):
            self.h_angle -= 4096 * 2
        if (self.v_angle > 1023 * 2):
            self.v_angle -= 2048 * 2
        self.x = self.radius * math.sin(self.v_angle * cpoint_angel_unit_c)
        t = self.radius * math.cos(self.v_angle * cpoint_angel_unit_c)
        self.y = t * math.sin(self.h_angle * cpoint_angel_unit_c)
        self.z = t * math.cos(self.h_angle * cpoint_angel_unit_c)

        self.ts_100us = ((raw_buf[6] & 0b01111111) << 7) | (
            (raw_buf[5] & 0b11111110) >> 1)
        self.scan_id = ((raw_buf[8] & 0b00000001) << 9) | (
            (raw_buf[7]) << 1) | ((raw_buf[6] & 0b10000000) >> 7)
        self.flags = (raw_buf[8] >> 1) & 0b00001111
        self.scan_idx = ((raw_buf[8] & 0b11100000) >> 5) | (raw_buf[9] << 3)

        self.ref = struct.unpack('H', raw_buf[10:12])[0]
        self.is_initialized = True

    def print_info(self):
        print('Point Info:')
        print('    x:%f,y:%f,z:%f,radius:%f,ts_100us:%d,ref:%d,flags:%d' % (
            self.x, self.y, self.z, self.radius, self.ts_100us, self.ref, self.flags))


class inno_cframe_header(object):
    def __init__(self, raw_buf=None):
        self.buf_size = 76
        self.is_initialized = False

        if raw_buf is None:
            self.version = 0
            self.flags = 0
            self.reserved_0 = 0
            self.checksum = 0
            self.longtitude = 0
            self.latitude = 0
            self.elevation = 0
            self.pose_yaw_angle = 0
            self.pose_pitch_angle = 0
            self.pose_roll_angle = 0
            self.idx = 0
            self.sub_idx = 0
            self.sub_seq = 0
            self.ts_us_start = 0
            self.ts_us_end = 0
            self.cframe_type = 0
            self.topic = 0
            self.item_number = 0
            self.conf_level = 0
            self.source_id = 0
            self.reserved_1 = 0
            self.reserved_2 = 0
            self.reserved_3 = 0
            self.timestamp_sync_type = 0
            self.reserved_4 = 0
            self.reserved_5 = 0
            self.reserved_6 = 0
        else:
            self.decode(raw_buf)

    def decode(self, raw_buf):
        # unpack 0~35 bytes
        self.version, self.flags, self.reserved_0, self.checksum,\
            self.longtitude, self.latitude, self.elevation,\
            self.pose_yaw_angle, self.pose_pitch_angle, self.pose_roll_angle,\
            self.idx, self.sub_idx, self.sub_seq = struct.unpack(
                "2BHI2i4hQ2h", raw_buf[0:36])

        # unpack 36~36+28 bytes
        self.ts_us_start, self.ts_us_end, self.cframe_type,\
            self.topic, self.item_number, self.conf_level = struct.unpack(
                "2di2IB", raw_buf[36:36+29])

        #
        self.source_id, self.reserved_1, self.reserved_2, self.reserved_3 = struct.unpack(
            "I3B", raw_buf[65:65+7])

        #
        self.timestamp_sync_type, self.reserved_4, self.reserved_5, self.reserved_6 = struct.unpack(
            "4B", raw_buf[72:72+4])

    def print_info(self):
        print('Cframe header Info:')
        print('    version    : %d,flags : %d,reserved_0 : %d,checksum : %d' %
              (self.version, self.flags, self.reserved_0, self.checksum))
        print('    longtitude : %d,latitude : %d,elevation : %d' %
              (self.longtitude, self.latitude, self.elevation))
        print('    pose_yaw_angle : %d,pose_pitch_angle : %d,pose_roll_angle : %d' % (
            self.pose_yaw_angle, self.pose_pitch_angle, self.pose_roll_angle))
        print('    idx : %d,sub_idx : %d,sub_seq : %d' %
              (self.idx, self.sub_idx, self.sub_seq))
        print('    ts_us_start : %f,ts_us_end : %f' %
              (self.ts_us_start, self.ts_us_end))
        print('    cframe_type : %d,topic : %d' %
              (self.cframe_type, self.topic))
        print('    item_number : %d,conf_level : %d,source_id : %d' %
              (self.item_number, self.conf_level, self.source_id))
        print('    reserved_1 : %d,reserved_2 : %d,reserved_3 : %d' %
              (self.reserved_1, self.reserved_2, self.reserved_3))
        print('    timestamp_sync_type : %d' % (self.timestamp_sync_type))
        print('    reserved_4 : %d,reserved_5 : %d,reserved_6 : %d' %
              (self.reserved_4, self.reserved_5, self.reserved_6))


class inno_cframe(object):
    def __init__(self, udp_payload=None):

        self.is_initialized = False

        if udp_payload is None:
            self.lidar_ctx = 0
            self.cframe_header = None
            self.points = []
        else:
            self.points = []
            self.decode(udp_payload)
        self.cframe_header.print_info()

    def decode(self, udp_payload):
        if udp_payload is not None:
            header = udp_payload[0:4]

            # Judge if is inn header
            if header != b'PS32':
                print('This packet is not a inn data packet!')
                return

            # Get inn payload
            inn_playload = udp_payload[4:]
            inno_cframe_header_len = 76
            point_offset = 4 + 4 + 4 + inno_cframe_header_len + 4
            lidar_ctx_buff = inn_playload[4:]
            inno_cframe_header_buff = inn_playload[4+4+4:]
            point_buff = inn_playload[point_offset:]

            len_c = struct.unpack("!i", udp_payload[4:8])[0]
            lidar_handle = struct.unpack("!i", udp_payload[8:12])[0]
            len_e = struct.unpack("!i", udp_payload[12:16])[0]
            len_g = struct.unpack(
                "!i", inn_playload[point_offset-4:point_offset])[0]

            # decode lidar ctx
            self.lidar_ctx = struct.unpack("i", lidar_ctx_buff[0:4])

            # decode cframe header
            self.cframe_header = inno_cframe_header(inno_cframe_header_buff)
            # self.cframe_header.print_info()

            # decode point
            if self.cframe_header.cframe_type == INNO_CFRAME_POINT:
                unit_size = 24  # 33
                unit_n = int(len_g/unit_size)
                self.points = []
                for c_point in range(0, unit_n):
                    pt = inno_point(
                        point_buff[unit_size*c_point:unit_size*(c_point+1)])
                    self.points.append(pt)
                    # not enter
                    print('INNO_CFRAME_POINT: ')
                    pt.print_info()
            # decode cpoint
            elif self.cframe_header.cframe_type == INNO_CFRAME_CPOINT:
                unit_size = 12  # 33
                unit_n = int(len_g/unit_size)
                # print(unit_n)
                self.points = []
                for c_point in range(0, unit_n):
                    pt = inno_cpoint(
                        point_buff[unit_size*c_point:unit_size*(c_point+1)])
                    pt.ts = pt.ts_100us + self.cframe_header.ts_us_start
                    # print (pt.ts, self.cframe_header.ts_us_end)
                    self.points.append(pt)
                    # pt.print_info()

    def print_info(self):
        print('Cframe Info:')
        print('    lidar_ctx : %d' % self.lidar_ctx)
        self.cframe_header.print_info()
        print('--------------------------------------------------------------------------------')
        point = 0
        for pt in self.points:
            print('Point index:%d' % point)
            point = point + 1
            pt.print_info()
            print(
                '--------------------------------------------------------------------------------')

class inno_lidar_frame(object):
    def __init__(self):
        self.idx = 0
        self.ts_us_start = 0
        self.ts_us_end = 0
        self.sub_seqs = []
        self.points = []
        self.is_initialized = False

    def init_by_first_cframe(self, cframe):
        self.idx = cframe.cframe_header.idx
        self.ts_us_start = cframe.cframe_header.ts_us_start
        self.ts_us_end = cframe.cframe_header.ts_us_end
        self.sub_seqs = [cframe.cframe_header.sub_seq]
        self.points = cframe.points
        self.is_initialized = True

    def add_cframe(self, cframe):
        if self.is_initialized:
            if cframe.cframe_header.idx == self.idx:
                self.ts_us_end = cframe.cframe_header.ts_us_end
                self.sub_seqs.append(cframe.cframe_header.sub_seq)
                self.points = self.points + cframe.points

    def reset(self):
        self.idx = 0
        self.ts_us_start = 0
        self.ts_us_end = 0
        self.sub_seqs = []
        self.points = []
        self.is_initialized = False


S_MAKE_FIRST_FRAME = 0
S_MAKE_FRAME = 1


class inn_pcap_converter(object):
    def __init__(self):
        self.pcap_path = None
        self.pcap_fid = None
        self.is_opened = False
        self.index = 0
        
    def open(self, pcap_path=None, root_path='./'):
        #self.pcap_fid = open(pcap_path, 'rb')
        self.pcap_path = pcap_path
        self.root_path = root_path
        falcon.init()

    def convert(self):
        # Read pcap with rdpcap
        print("Start to read pcap")
        # t1=time.time()
        # packets = rdpcap(self.pcap_path)
        # print("Done rdpcap， cost time(second):", time.time()-t1)
    
        # Loop all packet
        self.frame_lists = []
        fsm_state = S_MAKE_FIRST_FRAME

        # show pcd
        #vis = o3d.visualization.Visualizer()
        #vis.create_window(window_name='3D', visible=True)
        #ctr=vis.get_view_control()
        #ctr.set_constant_z_far(1000)
        #ctr.set_constant_z_near(0.01)
        #o3d_frame = None
        #mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        #    size=0.1)
        #vis.add_geometry(mesh_frame)
        raw = bytes()

        with scapy.utils.PcapReader(self.pcap_path) as packets:
            for pkt in packets:
                self.index = self.index + 1
                # Decode network layer,Only leave IP packet
                if pkt.payload.name != 'IP':
                    continue
                if (pkt.payload.payload.name == 'UDP' and pkt.payload.payload.dport == 8010 ) or pkt.payload.payload.name == 'Raw':
                    # Get udp_payload
                    udp_payload = pkt.payload.payload.load
                    # Decode the payload
                    if pkt.flags == 'DF' and pkt.payload.payload.name == 'UDP' and pkt.payload.payload.dport == 8010:
                        pcd = falcon.parse_inno_package(udp_payload, len(udp_payload), self.index)
                        if (pcd.shape[0] > 1):
                            filename = 'INNO_IDX_' + \
                                str(falcon.idx()) + '.pcd'
                            pcd_path = os.path.join(self.root_path, filename)
                            self.save_nparray_to_pcd(pcd, pcd_path)
                            print(falcon.idx(), falcon.ts_us_start(), pcd.shape)
                        continue
                    if pkt.flags == 'MF':
                        raw += udp_payload
                    elif pkt.flags.value == 0:
                        raw += udp_payload
                        pcd = falcon.parse_inno_package(raw, len(raw), self.index)
                        raw = bytes()
                        if(pcd.shape[0] > 1):
                            filename = 'INNO_IDX_' + \
                                str(falcon.idx()) + '.pcd'
                            pcd_path = os.path.join(self.root_path, filename)
                            self.save_nparray_to_pcd(pcd, pcd_path)
                            print(falcon.idx(), falcon.ts_us_start(), pcd.shape)
                            #show frame
                            # import open3d as o3d
                            # tmp = o3d.geometry.PointCloud()
                            # tmp.points = o3d.utility.Vector3dVector(pcd[:, :3])
                            # o3d.visualization.draw_geometries([tmp])
                    continue

    def save_inno_lidar_frame_to_pcd(self, inno_lidar_frame, root_path='./'):
        filename = 'INNO_IDX_' + \
            str(inno_lidar_frame.idx) + '_TIME_' + \
            str(int(inno_lidar_frame.ts_us_start))+'.pcd'
        fullpath = os.path.join(root_path, filename)
        self.save_inno_points_to_pcd(inno_lidar_frame.points, fullpath)

    def save_inno_points_to_pcd(self, inno_points, pcd_path):
        nparray = []
        for pt in inno_points:
            nparray.append([pt.x, pt.y, pt.z, pt.ref, pt.ts, 0])
        self.save_nparray_to_pcd(nparray, pcd_path)

    def save_nparray_to_pcd(self, nparray, pcd_path):
        saver = pcd_saver()
        saver.save(nparray, pcd_path)

def falcon_parse(input_file,output_dir,save_format):
    os.makedirs(output_dir, exist_ok=True)
    converter = inn_pcap_converter()
    converter.open(input_file, output_dir)
    converter.convert()

if __name__=="__main__":
    t1=time.time()
    parser = argparse.ArgumentParser(description="parse UDP packet data")
    parser.add_argument("-i", "--input", type=str, required=True, help='input pcap file')
    parser.add_argument("-o", "--output_dir", type=str, default="falcon_lidar", help='output pcd dir')
    args = parser.parse_args()
    falcon_parse(args.input, args.output_dir, "pcd")
    falcon.summary_print()
    print("Done parse, cost time(second):", time.time()-t1)
