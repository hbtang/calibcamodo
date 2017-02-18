#ifndef MAPPUBLISH_H
#define MAPPUBLISH_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "core/dataset.h"

namespace calibcamodo {

class Dataset;

class MapPublish{
public:
    MapPublish(Dataset* pDataset);
    ~MapPublish();

    Dataset* mpDataset;

    void run(int _numIter = 10, int flag = 0);

    void SetDataset(Dataset* pDataset) { mpDataset = pDataset; }

    void PublishMapPointsAruco();
    void PublishMapPointsOrb();
    void PublishKeyFrames();

private:

    ros::NodeHandle nh;
    ros::Publisher publisher;
    tf::TransformBroadcaster tfb;
    visualization_msgs::Marker mMps;
    visualization_msgs::Marker mKfs;
    visualization_msgs::Marker mVisGraph;
    visualization_msgs::Marker mOdoGraph;

    float mPointSize;
    float mCameraSize;
    float mScaleRatio;

};// class MapPublish

} // namespace calibcalmodo

#endif // MAPPUBLISH_H
