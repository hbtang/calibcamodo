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
#include "ros/mappublish.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace aruco;
using namespace calibcamodo;

int main(int argc, char **argv) {

    string strFolderPathMain = argv[1];
//    int numFrame = atoi(argv[2]);
//    double markerSize = atof(argv[3]);

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

    //! Solver
    cerr << "SolverOrb: init..." << endl;
    SolverOrb solverOrb(&datasetOrb);
    solverOrb.RefreshKfsPose();
    solverOrb.CreateMsrOdos();
    solverOrb.CreateMapPoints();
    mappublish.run(10, 1);
    cerr << "solverOrb: result.init = " << solverOrb.GetSe3cb() << endl;

    //! Do vslam
    cerr << "SolverOrb: do vslam..." << endl;
    solverOrb.OptimizeSlam();
    mappublish.run(10, 1);
    cerr << "solverOrb: result.vslam = " << solverOrb.GetSe3cb() << endl;

    //! Do vsclam
    cerr << "SolverOrb: do vsclam..." << endl;
    solverOrb.OptimizeSclam();
    mappublish.run(10, 1);
    cerr << "solverOrb: result.vsclam = " << solverOrb.GetSe3cb() << endl;

    return 0;
}
