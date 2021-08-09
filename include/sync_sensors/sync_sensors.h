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
    Sync_Sensors();
    virtual ~Sync_Sensors();

    void init1(message_filters::Synchronizer<MySyncPolicy1>& sync);
    void init2(message_filters::Synchronizer<MySyncPolicy2>& sync);

    void callback1(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::LaserScan::ConstPtr& scan);
    void callback2(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::PointCloud2::ConstPtr& points);

    sync_sensors::Sensors getData();

protected:
    sync_sensors::Sensors sensors_;
};

// Because of Linker Error, 'sync_sensors.cpp' code lines are written here.

Sync_Sensors::Sync_Sensors() {
}

Sync_Sensors::~Sync_Sensors() {
}

void Sync_Sensors::init1(message_filters::Synchronizer<MySyncPolicy1>& sync) {
    sync.registerCallback(boost::bind(&Sync_Sensors::callback1, this, _1, _2));
}

void Sync_Sensors::init2(message_filters::Synchronizer<MySyncPolicy2>& sync) {
    sync.registerCallback(boost::bind(&Sync_Sensors::callback2, this, _1, _2));
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

sync_sensors::Sensors Sync_Sensors::getData() {
    return sensors_;
}