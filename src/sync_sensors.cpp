#include <sys/time.h>
#include <iostream>

#include "sync_sensors/sync_sensors.h"


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

    sensors_.image = *data;
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