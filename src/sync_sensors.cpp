#include "sync_sensors/sync_sensors.h"


Sync_Sensors::Sync_Sensors(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
    Sync_Sensors::initialize();
}

Sync_Sensors::~Sync_Sensors() {
}

void Sync_Sensors::initialize() {
    nh_local_.param<int>("use_3d", use_3d_, 0);
    nh_local_.param<int>("is_test", is_test_, 0);
    nh_local_.param<std::string>("topic_cam", topic_cam_, "/image/image_raw");
    nh_local_.param<std::string>("topic_2d", topic_2d_, "/laser_scan");
    nh_local_.param<std::string>("topic_3d", topic_3d_, "/velodyne_points");

    ros::Publisher pub_sensors_ = nh_.advertise<sync_sensors::Sensors>("/sensors", 10);
    ros::Publisher pub_laser_ = nh_.advertise<sensor_msgs::LaserScan>("/sync_laser", 1);
    ros::Publisher pub_velo_ = nh_.advertise<sensor_msgs::PointCloud2>("/sync_velodyne", 1);
}

void Sync_Sensors::callback1(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::LaserScan::ConstPtr& scan) {
    sync_sensors::Sensors::Ptr data (new sync_sensors::Sensors);

    data->image = *image;
    data->scan = *scan;

    sensors_ = *data;
}

void Sync_Sensors::callback2(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::PointCloud2::ConstPtr& points) {
    sync_sensors::Sensors::Ptr data (new sync_sensors::Sensors);

    data->image = *image;
    data->points = *points;

    sensors_ = *data;
}

void Sync_Sensors::spin() {
    if (use_3d_ == 0) {
        message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, topic_cam_, 1);
        message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, topic_2d_, 1);

        message_filters::Synchronizer<MySyncPolicy1> sync(MySyncPolicy1(10), image_sub, scan_sub);

        sync.registerCallback(boost::bind(&Sync_Sensors::callback1, _1, _2));
        
        pub_sensors.publish(sensors_);

        if (is_test == 1) {
            pub_laser.publish(sensors_.scan);
        }
    } else {
        message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, topic_cam_, 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, topic_3d_, 1);

        message_filters::Synchronizer<MySyncPolicy2> sync(MySyncPolicy2(10), image_sub, points_sub);

        sync.registerCallback(boost::bind(&Sync_Sensors::callback2, _1, _2));
        
        pub_sensors.publish(sensors_);

        if (is_test == 1) {
            pub_velo.publish(sensors_.points);
        }
    }
}