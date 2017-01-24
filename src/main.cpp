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
    DatasetAruco datasetAruco;
    cerr << "Dataset: creating frames ..." << endl;
    datasetAruco.CreateFrames();
    cerr << "Dataset: creating keyframes ..." << endl;
    datasetAruco.CreateKeyFrames();
    cerr << "Dataset: creating measurement odometry ..." << endl;
    datasetAruco.CreateMsrOdos();
    cerr << "Dataset: creating aruco mark and measurements ..." << endl;
    datasetAruco.CreateMarks();

    cerr << "Dataset: dataset created." << endl << endl;

    //! Calibrate by SolverInitmk
    cerr << "SolverInitmk: init solver ..." << endl;
    SolverInitmk solverInitmk(&datasetAruco);
    solverInitmk.DoCalib();
    Se3 se3_cb = solverInitmk.GetSe3cb();
    cerr << "SolverInitmk: result = " << se3_cb << endl << endl;

    //! Calibrate by SolverOptmk
    cerr << "SolverOptmk: init solver ..." << endl;
    datasetAruco.InitAll(se3_cb);

    //! Debug: Show something here, with ros viewer
    MapPublish mappublish(&datasetAruco);
    mappublish.run();

    SolverOptMk solverOptmk(&datasetAruco);
    solverOptmk.SetSe3cb(se3_cb);
    solverOptmk.DoCalib();
    se3_cb = solverOptmk.GetSe3cb();
    cerr << "SolverOptmk: result = " << se3_cb << endl;

    mappublish.run();

    return 0;
}
