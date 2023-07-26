#include <ros/ros.h>
#include <inno_sdk/sdk_common/inno_lidar_api.h>
                                  
int main() {
    std::string LIDAR_IP;
    int LIDAR_PORT;
    bool USE_TCP;
    int UDP_PORT;
    ros::param::get("/lidar_config/ip", LIDAR_IP);
    ros::param::get("/lidar_config/port", LIDAR_PORT);
    ros::param::get("/lidar_config/use_tcp", USE_TCP);
    ros::param::get("/lidar_config/udp_port", UDP_PORT);

    int handle = inno_lidar_open_live("live",
                                      LIDAR_IP.c_str(),
                                      LIDAR_PORT,
                                      USE_TCP ? 
                                      INNO_LIDAR_PROTOCOL_PCS_TCP :
                                      INNO_LIDAR_PROTOCOL_PCS_UDP,
                                      UDP_PORT);
    inno_log_verify(handle > 0, "cannot open lidar");

    int j = 1;
    for (int i = 0; i < 11; i+=j) {
        if (i == 11) {
            j = -1;
        } else if (i == -11) {
            j = 1;
        }
        int ret = inno_lidar_set_roi(handle, 0, i);
        sleep(1);
    }
}