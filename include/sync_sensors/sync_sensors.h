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


class Sync_Sensors {
public:
    Sync_Sensors(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    virtual ~Sync_Sensors();

private:
    void initialize();

    void callback1(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::LaserScan::ConstPtr& scan);
    void callback2(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::PointCloud2::ConstPtr& points);

    // Variables
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> MySyncPolicy1;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy2;

    typedef message_filters::Synchronizer<MySyncPolicy1> Sync1;
    typedef message_filters::Synchronizer<MySyncPolicy2> Sync2;

    boost::shared_ptr<Sync1> sync1_;
    boost::shared_ptr<Sync2> sync2_;

    sync_sensors::Sensors sensors_;

    // Parameters
    int use_3d_;
    int is_test_;
    std::string topic_cam_;
    std::string topic_2d_;
    std::string topic_3d_;

    // Ros Variables
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::Publisher pub_sensors_;
    ros::Publisher pub_laser_;
    ros::Publisher pub_velo_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_;
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