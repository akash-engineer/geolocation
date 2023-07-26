How to use


python3 falcon_parser.py -i xxx.pcap -o xxxx
./parse_pcap -i xxx.pcap -o xxxx

xxx.pcap: input pcap file
xxxx    : output folder


Dependent Packages
 pip3 install scapy numpy pandas
 apt-get install libbz2-dev libpcap-dev libeigen3-dev
 python -m pip install pybind11

Error1:
parse_pcap.cpp:17:10: fatal error: pcap.h: No such file or directory
 #include <pcap.h>

Solution1:
sudo apt install libpcap-dev

Error2:
fatal error: Eigen/Core: No such file or directory
 #include <Eigen/Core>

Solution2:
sudo apt-get install libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

Error3:
fatal error: pybind11/pybind11.h: No such file or directory
 #include "pybind11/pybind11.h"

Solution3
python -m pip install pybind11
