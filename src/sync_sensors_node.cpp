
#include <ros/ros.h>
#include <string>

#include <sync_sensors/sync_sensors.h>


int main(int argc, char **argv) {
    Sync_Sensors ss;

    // parameters
    int use_3d;
    int is_test;
    std::string topic_cam;
    std::string topic_2d;
    std::string topic_3d;

    ros::init(argc, argv, "sync_sensors_node");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");
    ros::Publisher pub_sensors = nh.advertise<sync_sensors::Sensors>("/sensors", 10);
    ros::Publisher pub_laser = nh.advertise<sensor_msgs::LaserScan>("/sync_laser", 1);
    ros::Publisher pub_velo = nh.advertise<sensor_msgs::PointCloud2>("/sync_velodyne", 1);

    n.param<int>("use_3d", use_3d, 0);
    n.param<int>("is_test", is_test, 0);
    n.param<std::string>("topic_cam", topic_cam, "/image");
    n.param<std::string>("topic_2d", topic_2d, "/laser_scan");
    n.param<std::string>("topic_3d", topic_3d, "/velodyne_points");

    if (use_3d == 0) {
        message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, topic_cam, 1);
        message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, topic_2d, 1);

        message_filters::Synchronizer<MySyncPolicy1> sync(MySyncPolicy1(10), image_sub, scan_sub);

        sync.registerCallback(boost::bind(&Sync_Sensors::callback1, &ss, _1, _2));
        ss.init1(sync);
        
        pub_sensors.publish(ss.getData());

        if (is_test == 1) {
            pub_laser.publish(ss.getData().scan);
        }
    } else {
        message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, topic_cam, 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, topic_3d, 1);

        message_filters::Synchronizer<MySyncPolicy2> sync(MySyncPolicy2(10), image_sub, points_sub);

        sync.registerCallback(boost::bind(&Sync_Sensors::callback2, &ss, _1, _2));
        ss.init2(sync);
        
        pub_sensors.publish(ss.getData());

        if (is_test == 1) {
            pub_velo.publish(ss.getData().points);
        }
    }

    ros::spin();

    return 0;
}