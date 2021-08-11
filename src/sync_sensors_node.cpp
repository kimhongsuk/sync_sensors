#include "sync_sensors/sync_sensors.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_sensors_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    ROS_INFO("[Sync Sensors]: Initializing node");
    Sync_Sensors ss(nh, nh_local);

    ros::spin();

    return 0;
}