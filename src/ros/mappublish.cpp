#include "mappublish.h"
#include "core/config.h"
#include "core/frame.h"
#include "core/mapmark.h"
#include "core/mappoint.h"
#include "core/adapter.h"

using namespace std;
using namespace cv;

namespace calibcamodo {

MapPublish::MapPublish(Dataset* pDataset){
    mpDataset = pDataset;

    const char* MAP_FRAME_ID = "/calibcamodo/World";

    const char* CAMERA_NAMESPACE = "Camera";
    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    const char* MAPPOINTS_NAMESPACE = "MapPoints";

    const char* GRAPH_NAMESPACE = "Graph";
    const char* VISGRAPH_NAMESPACE = "VisGraph";
    const char* ODOGRAPH_NAMESPACE = "OdoGraph";

    // Set Scale Ratio
    mScaleRatio = Config::MAPPUB_SCALE_RATIO;
    // Set Camera Size
    mCameraSize = 0.2;
    // Set MapPoint Size
    mPointSize = 0.1;

    //Configure KFs
    mKfs.header.frame_id = MAP_FRAME_ID;
    mKfs.ns = KEYFRAMES_NAMESPACE;
    mKfs.id = 0;
    mKfs.type = visualization_msgs::Marker::LINE_LIST;
    mKfs.scale.x = 0.05;
    mKfs.scale.y = 0.05;
    mKfs.pose.orientation.w = 1.0;
    mKfs.action = visualization_msgs::Marker::ADD;
    mKfs.color.r = 0.0;
    mKfs.color.g = 0.0;
    mKfs.color.b = 0.0;
    mKfs.color.a = 1.0;

    //Configure MPs
    mMps.header.frame_id = MAP_FRAME_ID;
    mMps.ns = MAPPOINTS_NAMESPACE;
    mMps.id = 2;
    mMps.type = visualization_msgs::Marker::POINTS;
    mMps.scale.x = mPointSize;
    mMps.scale.y = mPointSize;
    mMps.pose.orientation.w = 1.0;
    mMps.action = visualization_msgs::Marker::ADD;
    mMps.color.r = 0.0;
    mMps.color.g = 0.0;
    mMps.color.b = 1.0;
    mMps.color.a = 1.0;

    //Configure Visual Graph
    mVisGraph.header.frame_id = MAP_FRAME_ID;
    mVisGraph.ns = VISGRAPH_NAMESPACE;
    mVisGraph.id = 4;
    mVisGraph.type = visualization_msgs::Marker::LINE_LIST;
    mVisGraph.scale.x = 0.05;
    mVisGraph.scale.y = 0.05;
    mVisGraph.pose.orientation.w = 1.0;
    mVisGraph.action = visualization_msgs::Marker::ADD;
    mVisGraph.color.r = 0.0;
    mVisGraph.color.g = 1.0;
    mVisGraph.color.b = 0.0;
    mVisGraph.color.a = 1.0;

    // Configure Odometry Constraint Graph
    mOdoGraph.header.frame_id = MAP_FRAME_ID;
    mOdoGraph.ns = ODOGRAPH_NAMESPACE;
    mOdoGraph.id=6;
    mOdoGraph.type = visualization_msgs::Marker::LINE_LIST;
    mOdoGraph.scale.x = 0.05;
    mOdoGraph.scale.y = 0.05;
    mOdoGraph.pose.orientation.w=1.0;
    mOdoGraph.action=visualization_msgs::Marker::ADD;
    mOdoGraph.color.r = 0.0;
    mOdoGraph.color.g = 0.0;
    mOdoGraph.color.b = 0.0;
    mOdoGraph.color.a = 1.0;


    tf::Transform tfT;
    tfT.setIdentity();
    tfb.sendTransform(tf::StampedTransform(tfT,ros::Time::now(),"/calibcamodo/World","/calibcamodo/Camera"));

    publisher = nh.advertise<visualization_msgs::Marker>("calibcamodo/Map", 10);

    publisher.publish(mKfs);
    publisher.publish(mMps);
    publisher.publish(mVisGraph);
    publisher.publish(mOdoGraph);
}

MapPublish::~MapPublish(){}

void MapPublish::PublishKeyFrames(){
    mKfs.points.clear();
    mOdoGraph.points.clear();

    //Camera is a pyramid. Define in camera coordinate system
    float d = mCameraSize;
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    // Draw KFs
    set<PtrKeyFrame> sKFsAll = mpDataset->GetKfSet();
    for(auto ptr : sKFsAll) {
        PtrKeyFrame pKf = ptr;
        cv::Mat Twc = pKf->GetPoseCamera().T();
        Twc.at<float>(0,3) = Twc.at<float>(0,3)/mScaleRatio;
        Twc.at<float>(1,3) = Twc.at<float>(1,3)/mScaleRatio;
        Twc.at<float>(2,3) = Twc.at<float>(2,3)/mScaleRatio;

        cv::Mat ow = Twc*o;
        cv::Mat p1w = Twc*p1;
        cv::Mat p2w = Twc*p2;
        cv::Mat p3w = Twc*p3;
        cv::Mat p4w = Twc*p4;

        geometry_msgs::Point msgs_o, msgs_p1, msgs_p2, msgs_p3, msgs_p4;

        msgs_o.x = ow.at<float>(0);
        msgs_o.y = ow.at<float>(1);
        msgs_o.z = ow.at<float>(2);
        msgs_p1.x = p1w.at<float>(0);
        msgs_p1.y = p1w.at<float>(1);
        msgs_p1.z = p1w.at<float>(2);
        msgs_p2.x = p2w.at<float>(0);
        msgs_p2.y = p2w.at<float>(1);
        msgs_p2.z = p2w.at<float>(2);
        msgs_p3.x = p3w.at<float>(0);
        msgs_p3.y = p3w.at<float>(1);
        msgs_p3.z = p3w.at<float>(2);
        msgs_p4.x = p4w.at<float>(0);
        msgs_p4.y = p4w.at<float>(1);
        msgs_p4.z = p4w.at<float>(2);

        mKfs.points.push_back(msgs_o);
        mKfs.points.push_back(msgs_p1);
        mKfs.points.push_back(msgs_o);
        mKfs.points.push_back(msgs_p2);
        mKfs.points.push_back(msgs_o);
        mKfs.points.push_back(msgs_p3);
        mKfs.points.push_back(msgs_o);
        mKfs.points.push_back(msgs_p4);
        mKfs.points.push_back(msgs_p1);
        mKfs.points.push_back(msgs_p2);
        mKfs.points.push_back(msgs_p2);
        mKfs.points.push_back(msgs_p3);
        mKfs.points.push_back(msgs_p3);
        mKfs.points.push_back(msgs_p4);
        mKfs.points.push_back(msgs_p4);
        mKfs.points.push_back(msgs_p1);


    }

    //Draw odometry graph
    set<PtrMsrSe2Kf2Kf> setMsrOdo = mpDataset->GetMsrOdoSet();
    for (auto ptr : setMsrOdo) {
        PtrMsrSe2Kf2Kf pMsrOdo = ptr;
        PtrKeyFrame pKfHead = pMsrOdo->pKfHead;
        PtrKeyFrame pKfTail = pMsrOdo->pKfTail;
        if (pKfHead == nullptr || pKfTail == nullptr) {
            return;
        }

        Mat TwcHead = pKfHead->GetPoseCamera().T();
        Mat TwcTail = pKfTail->GetPoseCamera().T();

        geometry_msgs::Point msgs_o_head;
        msgs_o_head.x = TwcHead.at<float>(0,3)/mScaleRatio;
        msgs_o_head.y = TwcHead.at<float>(1,3)/mScaleRatio;
        msgs_o_head.z = TwcHead.at<float>(2,3)/mScaleRatio;
        geometry_msgs::Point msgs_o_tail;
        msgs_o_tail.x = TwcTail.at<float>(0,3)/mScaleRatio;
        msgs_o_tail.y = TwcTail.at<float>(1,3)/mScaleRatio;
        msgs_o_tail.z = TwcTail.at<float>(2,3)/mScaleRatio;

        mOdoGraph.points.push_back(msgs_o_head);
        mOdoGraph.points.push_back(msgs_o_tail);
    }

    mKfs.header.stamp = ros::Time::now();
    mOdoGraph.header.stamp = ros::Time::now();

    publisher.publish(mKfs);
    publisher.publish(mOdoGraph);
}

void MapPublish::PublishMapPointsOrb() {

    mMps.points.clear();
    mVisGraph.points.clear();

    // set mappoints
    set<PtrMapPoint> setMp = mpDataset->GetMpSet();
    for (auto ptr : setMp) {
        PtrMapPoint pMp = ptr;
        geometry_msgs::Point msg_p;
        msg_p.x = pMp->GetPos().x/mScaleRatio;
        msg_p.y = pMp->GetPos().y/mScaleRatio;
        msg_p.z = pMp->GetPos().z/mScaleRatio;
        mMps.points.push_back(msg_p);
    }

    mMps.header.stamp = ros::Time::now();
    publisher.publish(mMps);

    //    mVisGraph.header.stamp = ros::Time::now();
    //    publisher.publish(mVisGraph);
}


void MapPublish::PublishMapPointsAruco() {

    mMps.points.clear();
    mVisGraph.points.clear();

    set<PtrMapMark> setMk = mpDataset->GetMkSet();
    for (auto ptr : setMk) {
        PtrMapMark pMk = ptr;
        geometry_msgs::Point msg_p;
        msg_p.x = pMk->GetPose().tvec.at<float>(0)/mScaleRatio;
        msg_p.y = pMk->GetPose().tvec.at<float>(1)/mScaleRatio;
        msg_p.z = pMk->GetPose().tvec.at<float>(2)/mScaleRatio;
        mMps.points.push_back(msg_p);
    }

    set<PtrMsrPt3Kf2Mk> setMsrMk = mpDataset->GetMsrMkAll();
    for (auto ptr : setMsrMk) {
        PtrMsrPt3Kf2Mk pMsrMk = ptr;
        PtrKeyFrame pKf = pMsrMk->pKf;
        PtrMapMark pMk = pMsrMk->pMk;


        geometry_msgs::Point msgs_o_kf;
        msgs_o_kf.x = pKf->GetPoseCamera().tvec.at<float>(0)/mScaleRatio;
        msgs_o_kf.y = pKf->GetPoseCamera().tvec.at<float>(1)/mScaleRatio;
        msgs_o_kf.z = pKf->GetPoseCamera().tvec.at<float>(2)/mScaleRatio;
        geometry_msgs::Point msgs_o_mk;
        msgs_o_mk.x = pMk->GetPose().tvec.at<float>(0)/mScaleRatio;
        msgs_o_mk.y = pMk->GetPose().tvec.at<float>(1)/mScaleRatio;
        msgs_o_mk.z = pMk->GetPose().tvec.at<float>(2)/mScaleRatio;

        mVisGraph.points.push_back(msgs_o_kf);
        mVisGraph.points.push_back(msgs_o_mk);
    }



    mMps.header.stamp = ros::Time::now();
    mVisGraph.header.stamp = ros::Time::now();
    publisher.publish(mMps);
    publisher.publish(mVisGraph);
}


void MapPublish::run(int _numIter, int _flag) {
    ros::Rate rate(5);
    while (ros::ok() && _numIter != 0) {
        PublishKeyFrames();

        switch(_flag) {
        case 0:
            PublishMapPointsAruco();
            break;

        case 1:
            PublishMapPointsOrb();
            break;

        default:
            break;
        }

        rate.sleep();
        _numIter--;
    }
}

}
