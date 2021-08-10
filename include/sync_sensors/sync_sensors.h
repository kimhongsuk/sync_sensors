#include <ros/ros.h>
#include <string>
#include <sys/time.h>
#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sync_sensors/Sensors.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> MySyncPolicy1;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy2;

class Sync_Sensors {
public:
    Sync_Sensors(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    virtual ~Sync_Sensors();

private:
    initialize();

    void callback1(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::LaserScan::ConstPtr& scan);
    void callback2(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::PointCloud2::ConstPtr& points);

    // Variables
    sync_sensors::Sensors sensors_;

    // Parameters
    int use_3d_;
    int is_test_;
    std::string topic_cam_;
    std::string topic_2d_;
    std::string topic_3d_;

    // Ros Variables
    ros::NodeHandle nh_;
    ros::NodeHandle n_local_;
    ros::Publisher pub_sensors_;
    ros::Publisher pub_laser_;
    ros::Publisher pub_velo_;
};

// Because of Linker Error, 'sync_sensors.cpp' code lines are written here.

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