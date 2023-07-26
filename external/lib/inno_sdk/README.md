# Files/Dirs Description


| File / directory |                       | Description                                 |
| ---------------- | --------------------- | ------------------------------------------- |
| README           |                       | This file                                   |
| SDK_VERSION      |                       | SDK VERSION file                            |
| Makefile         |                       | Makefile to re-compile source code          |
| src/sdk_common/  | inno_lidar_api.h      | key include-file describing inno_lidar APIs |
|                  | inno_lidar_packet.h   | key include-file describing packets struct  |
|                  | inno_lidar_packet_utils.h  | packets utilities                      |
| src/             | sdk_client            | source files to build sdk_client library (run make) |
| src/utils/       |                       | source files to build utility library (run make)    |
|                  | inno_lidar_log.h      | key include-file describing log APIs                |
|                  | sdk_common            | common files to build sdk_client library (run make) |
| lib/             |                       | library files                                       |
| apps/pcs/        |                       | pointcloud static linked executables                |
|                  |                       | and source code (run `make dynamic`)                |
|                  |                       | It also contains inno_pc_npy.py that can parse      |
|                  |                       | .inno_pc_npy file which is captured                 |
|                  |                       | by inno_pc_client and generate xyzi .pcd files.     |
| apps/example/    |                       | source files to build example client (run make)     |
| apps/parse/      |                       | source files to build utility to parse .pcap file   |

Run `make clean && make` in the top directory to re-compile source code.

To make the apps/parse/ source, you may need to install the following packages:

    sudo apt-get install python3-dev libpcap-dev libeigen3-dev
    sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
