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
    cerr << "Dataset: dataset created." << endl << endl;

    //! Init mappublisher with ros rviz
    MapPublish mappublish(&datasetAruco);


    //! Calibrate by SolverInitmk
    cerr << "SolverInitmk: init solver ..." << endl;
    SolverInitmk solverInitmk(&datasetAruco);

    cerr << "SolverInitmk: creating measurement odometry ..." << endl;
    solverInitmk.CreateMsrOdos();

    cerr << "SolverInitmk: creating aruco mark and measurements ..." << endl;
    solverInitmk.CreateMarks();

    cerr << "SolverInitmk: do calibration with inimk algorithm ..." << endl;
    solverInitmk.DoCalib();

    cerr << "SolverInitmk: result = " << solverInitmk.GetSe3cb() << endl << endl;

    //! Calibrate by SolverOptmk
    cerr << "SolverOptMk: init solver ..." << endl;
    SolverOptMk solverOptmk(&datasetAruco);
    solverOptmk.SetSe3cb(solverInitmk.GetSe3cb());

    cerr << "SolverOptMk: refresh pose of all kfs and mps in dataset ..." << endl;
    solverOptmk.RefreshAllPose();
    mappublish.run();

    cerr << "SolverOptMk: do calibration with optmk algorithm ..." << endl;
    solverOptmk.DoCalib();
    cerr << "SolverOptMk: result = " << solverOptmk.GetSe3cb() << endl;
    mappublish.run();

    return 0;
}
