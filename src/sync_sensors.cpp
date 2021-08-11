#include "sync_sensors/sync_sensors.h"


Sync_Sensors::Sync_Sensors(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
    Sync_Sensors::initialize();
}

Sync_Sensors::~Sync_Sensors() {
}

void Sync_Sensors::initialize() {
    nh_local_.param<int>("use_3d", use_3d_, 0);
    nh_local_.param<int>("is_test", is_test_, 0);
    nh_local_.param<std::string>("topic_cam", topic_cam_, "/usb_cam/image_raw");
    nh_local_.param<std::string>("topic_2d", topic_2d_, "/laser_scan");
    nh_local_.param<std::string>("topic_3d", topic_3d_, "/velodyne_points");

    pub_sensors_ = nh_.advertise<sync_sensors::Sensors>("/sync_sensors", 10);
    pub_laser_ = nh_.advertise<sensor_msgs::LaserScan>("/sync_laser", 1);
    pub_velo_ = nh_.advertise<sensor_msgs::PointCloud2>("/sync_velodyne", 1);

    image_sub_.subscribe(nh_, topic_cam_, 1);
    scan_sub_.subscribe(nh_, topic_2d_, 1);
    // points_sub_.subscribe(nh_, topic_3d_, 1);

    sync1_.reset(new Sync1(MySyncPolicy1(10), image_sub_, scan_sub_));
    sync1_->registerCallback(boost::bind(&Sync_Sensors::callback1, this, _1, _2));

    // sync2_.reset(new Sync2(MySyncPolicy2(10), image_sub_, scan_sub_));
    // sync2_->registerCallback(boost::bind(&Sync_Sensors::callback2, this, _1, _2));

        ROS_INFO("[Sync Sensors]: Initialized");
}

void Sync_Sensors::callback1(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::LaserScan::ConstPtr& scan) {
    sync_sensors::Sensors::Ptr data (new sync_sensors::Sensors);

    data->image = *image;
    data->scan = *scan;

    pub_sensors_.publish(*data);

    if (is_test_ == 1) {
        ROS_INFO("[Sync Sensors]: Sync_Sensors Success");

        std::cout << image->header.stamp << std::endl;
        std::cout << scan->header.stamp << std::endl;

        pub_laser_.publish(data->scan);
    }
}

void Sync_Sensors::callback2(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::PointCloud2::ConstPtr& points) {
    sync_sensors::Sensors::Ptr data (new sync_sensors::Sensors);

    data->image = *image;
    data->points = *points;

    pub_sensors_.publish(*data);

    if (is_test_ == 1) {
        ROS_INFO("[Sync Sensors]: Sync_Sensors Success");

        std::cout << image->header.stamp << std::endl;
        std::cout << points->header.stamp << std::endl;
        
        pub_velo_.publish(data->points);
    }
}