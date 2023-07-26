===============
File Formats
===============
inno_raw file: Innovusion proprietary RAW data file. Only inno_pc_server can process it.

inno_pc/inno_pc_xyz file: Innovusion proprietary pointcloud file. Consists of many InnoDataPacket and InnoStatusPacket. It is basically concatenation the inno_pc_server's output. inno_pc_client and example/get_pcd can process it. inno_pc_xyz files use cartesian coordinates and inno_pc files use spherical coordinates.

inno_pc_npy file: Innovusion proprietary pointcloud file. inno_pc_npy.py can process it.

===============
Common Tasks
===============
- Connect to a live lidar and view the pointcloud in browser:
  ./inno_pc_client --lidar-ip 172.168.1.10 --lidar-port 8010 --lidar-udp-port 13400 -V

- Connect to a live lidar and record the pointcloud in out1.inno_pc_xyz file:
  ./inno_pc_client --lidar-ip 172.168.1.10 --lidar-port 8010 --lidar-udp-port 13400 --record-inno-pc-filename out1 --use-xyz 1

- Connect to a live lidar and record the pointcloud in out1.inno_pc file:
  ./inno_pc_client --lidar-ip 172.168.1.10 --lidar-port 8010 --lidar-udp-port 13400 --record-inno-pc-filename out1

- Read from an inno_raw file and view the pointcloud in browser, repeat 10 times:
  ./inno_pc_server --file input.inno_raw --speed 15 --rewind 10 -V

- Read from an inno_pc file and view the pointcloud in browser, repeat 10 times:
  ./inno_pc_client --file input.inno_pc --speed 15 --rewind 10 -V

- Read from an inno_raw file using inno_pc_server to emulate a live Lidar. Read from the emulator using inno_pc_client and view the pointcloud in browser:
  ./inno_pc_server --file input.inno_raw --speed 15 --rewind 100 --tcp-port 8010 --udp-ip 127.0.0.1 --udp-port 13400 --udp-port-status 13400 --udp-port-message 13400
  (on a separate terminal)
  ./inno_pc_client --lidar-ip 127.0.0.1 --lidar-port 8010 --lidar-udp-port 13400 -V

- Convert an inno_raw file to an inno_pc file
  ./inno_pc_server --file input.inno_raw --record-inno-pc-filename out1

- Convert an inno_raw file to an rosbag file
  ./inno_pc_server --file input.inno_raw --record-inno-pc-filename output --speed 14 --record-rosbag-filename output --record-rosbag-size-in-m -1

- Extract one frame from an inno_pc file and save to a pcd file
  ../example/get_pcd --inno-pc-filename input.inno_pc --pcd-filename output.pcd

- view a pcd file
  pcl_viewer input.pcd

- capture a pcd/bag/inno_pc/inno_raw file from a Lidar emulator (or a live Lidar)
  curl "127.0.0.1:8010/capture/?type=pcd&duration=15" -O -J
  curl "127.0.0.1:8010/capture/?type=bag&duration=1000" -O -J
  curl "127.0.0.1:8010/capture/?type=inno_pc&duration=1000" -O -J
  curl "127.0.0.1:8010/capture/?type=inno_raw&duration=1000" -O -J

- debug performance/latency problems
  curl "127.0.0.1:8010/command/?get_support"
  curl "127.0.0.1:8010/command/?get_system_stats"
  curl "127.0.0.1:8010/command/?get_cpu_signal"
  curl "127.0.0.1:8010/command/?get_cpu_angle"
  curl "127.0.0.1:8010/command/?get_cpu_n0"
  curl "127.0.0.1:8010/command/?get_cpu_n1"
  curl "127.0.0.1:8010/command/?get_stage_deliver"

- other useful curl commands
  curl "http://127.0.0.1:8010/command/?get_usage"

output:
http://<LIDAR-IP>:8010/capture/?type=<TYPE>&duration=<DURATION>
 <TYPE> is one of the following: pcd bag inno_pc inno_raw
 <DURATION> is in number of frames (for pcd and inno_pc) or MBytes
 example: curl "http://172.168.1.10:8010/capture/?type=pcd&duration=15" -O -J

http://<LIDAR-IP>:8010/command/?<COMMAND>
 <COMMAND> is one of the following:
 get_usage get_commands
 get_sw_version get_fw_version get_sdk_version get_sdk_build_tag get_sdk_build_time
 get_sn get_model get_mode_status get_temperature get_detector_temps get_motor_speeds
 get_roi get_frame_rate get_reflectance_mode get_return_mode
 get_command_line get_debug get_status_interval_ms get_udp_ports_ip get_udp_ip
 get_uptime get_pid get_system_stats get_output_stats
 get_cpu_read get_cpu_signal get_cpu_angle get_cpu_n0 get_cpu_n1 get_cpu_deliver
 get_stage_read get_stage_signal get_stage_angle get_stage_n0 get_stage_n1 get_stage_deliver

===============
WebGl Viewer
===============
- when launch inno_pc_server or inno_pc_client, add '-V' will automatically launch WebGL viewer
- only chrome is supported
- keys
     <home> : reset camera
     <end> : reset camera and connection
     <w/s/e/q/d/a> : move camera (pan)
     <arrow_key> : move camera (rotate around pivot)
     <number_N> : load camear position from bookmark N
              + <left_shift> : fast transition
              + <right_shift> : slow transition
              + <b> : save current camera position to bookmark N
              + <v> : enter tracking mode N
     <-> <+> decreae/increase point decay by 0.1 second (default is 0.3 seconds)
     <o> highlight channel (round-robin)
     <,> : increase point size (non-roi)
       + <left_shift> : reduce non-roi point size (non-roi)
       + <shift_shift> : reset non-roi point size (non-roi)
     <.> : increase roi point size
       + <left_shift> : reduce roi point size
       + <shift_shift> : reset roi point size
     <k> : rotate color scheme (non-roi)
       + <left_shift> : reverse rotate color scheme (non-roi)
       + <shift_shift> : rest to default color scheme (non-roi)
     <l> : rotate color scheme (roi)
       + <left_shift> : reverse rotate color scheme (roi)
       + <shift_shift> : rest to default color scheme (roi)
     <r> + <left_shift> : enable/disable roi display mode, i.e. differnt color scheme for roi
     <space> : enable/disable camera random move

- mouse
    mouse drag (left or middle or right button while move)
    hold <left-shif> or/and <left-ctrl> or/and <left-alt> will have different effect
    hold <z> while left-click and drag: select points, the information of the selected points will be shown in the 'developer tool/console'

===============
ROS Driver (docker mode)
===============
(with internal server on, the ros driver is just a thin client)
  ./launch-docker.py --lidar-ip <LIDAR_ID> --processed

(with internal server off, the raw data processing is done by the host that runs the ros driver)
  ./launch-docker.py --lidar-ip <LIDAR_ID>

===============
ROS Driver (native mode)
===============
(with internal server on, the ros driver is just a thin client)
  roslaunch innovusion_pointcloud innovusion_points.launch device_ip:=<LIDAR_IP> port:=8010 udp_port:=8010 processed:=1

(with internal server off, the raw data processing is done by the host that runs the ros driver)
  roslaunch innovusion_pointcloud innovusion_points.launch device_ip:=<LIDAR_IP> processed:=0
