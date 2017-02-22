#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>

#include "core/dataset.h"
#include "core/frame.h"
#include "core/measure.h"
#include "core/mapmark.h"
#include "core/adapter.h"
#include "core/type.h"
#include "core/config.h"
#include "core/makeronline_orb.h"
#include "core/solveronline_ekf.h"

#include "ros/mappublish.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

#include "core/math_opt.h"

using namespace std;
using namespace cv;
using namespace calibcamodo;

int main(int argc, char **argv) {

    string strFolderPathMain = argv[1];

    //! Init ros
    ros::init(argc, argv, "pub");
    ros::start();
    ros::NodeHandle nh;
    ros::Rate rate(100);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw",1);

    //! Init config
    Config::InitConfig(strFolderPathMain);

    //! Init dataset
    cerr << "DatasetOrb: init ..." << endl;
    DatasetOrb datasetOrb;
    cerr << "DatasetOrb: creating frames ..." << endl;
    datasetOrb.CreateFrames();
    cerr << "DatasetOrb: creating keyframes ..." << endl;
    datasetOrb.CreateKeyFrames();
    cerr << "DatasetOrb: dataset created." << endl << endl;

    //! Init mappublisher with ros rviz
    MapPublish mappublish(&datasetOrb);

    // DEBUG...
    MakerOnlineOrb makerOnlineOrb(&datasetOrb);
    SolverOnlineEkf solverOnlineEkf(&datasetOrb);

    int count = 0;
    while(makerOnlineOrb.DoMakeOnce()) {
        solverOnlineEkf.DoFilterOnce();
        cerr << "loop count = " << count++ << endl;
        if(count % 3 == 0)
            mappublish.run(1, 1);
    }

    return 0;
}
