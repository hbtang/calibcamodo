#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "core/dataset.h"
#include "core/frame.h"
#include "core/measure.h"
#include "core/mapmark.h"
#include "core/maker_aruco.h"
#include "core/solver_initmk.h"
#include "core/solver_optmk.h"
#include "core/adapter.h"
#include "core/type.h"
#include "core/config.h"

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

    // Init ros
    ros::init(argc, argv, "pub");
    ros::start();
    ros::NodeHandle nh;
    ros::Rate rate(100);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw",1);

    // Init config
    Config::InitConfig(strFolderPathMain);

    // Init dataset
    DatasetAruco datasetAruco;
    cerr << "Dataset: creating frames ..." << endl;
    datasetAruco.CreateFrames();
    cerr << "Dataset: creating keyframes ..." << endl;
    datasetAruco.CreateKeyFrames();
    cerr << "Dataset: dataset created." << endl << endl;

    // Init mappublisher with ros rviz
    MapPublish mappublish(&datasetAruco);


    // Set the problem in dataset by makerAruco
    MakerAruco makerAruco(&datasetAruco);
    makerAruco.DoMake();
    mappublish.run(10, 0);

    // Calibrate by SolverInitMk
    SolverInitMk solverInitMk(&datasetAruco);
    solverInitMk.DoCalib();
    cerr << "solverInitMk: result = " << datasetAruco.GetCamOffset() << endl << endl;
    makerAruco.InitKfMkPose();
    mappublish.run(10, 0);

    // Calibrate by SolverOptMk
    SolverOptMk solverOptMk(&datasetAruco);
    solverOptMk.DoCalib();
    cerr << "solverInitMk: result = " << datasetAruco.GetCamOffset() << endl << endl;
    mappublish.run(10, 0);

    return 0;
}
