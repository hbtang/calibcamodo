#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "dataset.h"
#include "frame.h"
#include "measure.h"
#include "mark.h"
#include "solver.h"
#include "adapter.h"
#include "type.h"
#include "config.h"

//#include <ros/ros.h>
//#include <nav_msgs/Odometry.h>
//#include <std_msgs/Header.h>
//#include <std_msgs/Float32MultiArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/Image.h>
//#include <visualization_msgs/Marker.h>
////#include <tf/transform_broadcaster.h>
////#include <tf/transform_datatypes.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace aruco;
using namespace calibcamodo;

int main(int argc, char **argv) {

    string strFolderPathMain = argv[1];
    int numFrame = atoi(argv[2]);
    double markerSize = atof(argv[3]);

//    //! Init ros
//    ros::init(argc, argv, "pub");
//    ros::start();
//    ros::NodeHandle nh;
//    ros::Rate rate(100);
//    image_transport::ImageTransport it(nh);
//    image_transport::Publisher pub = it.advertise("/camera/image_raw",1);

    //! Init config
    Config::InitConfig(strFolderPathMain, numFrame, markerSize);

    //! Init dataset
    Dataset dataset;
    dataset.CreateFrame();
    dataset.CreateKeyFrame();
    dataset.CreateMarkMeasure();    

    //! Init solver
    Solver solver(&dataset);

    //! Do calibrate with "initmk" algorithm
    solver.CalibInitMk(dataset.GetMsrMk(), dataset.GetMsrOdo());
    Se3 se3bc_initmk = solver.GetResult();
    cerr << "initmk se3bc: " << se3bc_initmk << endl;

    //! Do calibrate with "optmk" algorithm
    dataset.InitAll(solver.GetResult());
    solver.CalibOptMk(dataset.GetMsrMk(), dataset.GetMsrOdo());
    Se3 se3bc_optmk = solver.GetResult();
    cerr << "optmk se3bc: " << se3bc_optmk << endl;

    //! DEBUG: show something here ...
//    for (auto pair : dataset.GetKeyFrameMap()) {
//        auto pf = pair.second;
//        Mat img = pf->GetImg();

//        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
//        pub.publish(msg);

//        cerr << pf->GetId() << " "
//             << pf->GetOdo().x << " "
//             << pf->GetOdo().y << " "
//             << pf->GetOdo().theta << endl;

//        rate.sleep();
//    }
//    for (auto ptr_measure : dataset.GetMeasureKf2AMk()) {
//        PtrKeyFrame pKf = ptr_measure->pKf;
//        PtrArucoMark pAMk = ptr_measure->pAMk;
//        cerr << pKf->GetId() << " " << pAMk->GetId() << " " << endl;
//        cerr << ptr_measure->measure << endl;
//    }
//    for (auto ptr_measure : dataset.GetMeasureKf2KfOdo()) {
//        PtrKeyFrame pKfHead = ptr_measure->pKfHead;
//        PtrKeyFrame pKfTail = ptr_measure->pKfTail;
//        cerr << pKfHead->GetId() << " " << pKfTail->GetId() << " " << endl;
//        cerr << ptr_measure->measure << endl;
//    }

    return 0;
}
