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
    int numFrame = atoi(argv[2]);
    double markerSize = atof(argv[3]);

    //! Init ros
    ros::init(argc, argv, "pub");
    ros::start();
    ros::NodeHandle nh;
    ros::Rate rate(100);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw",1);

    //! Init config
    Config::InitConfig(strFolderPathMain, numFrame, markerSize);

    //! Init dataset
    Dataset dataset;
    cerr << "Dataset: creating frames ..." << endl;
    dataset.CreateFrame();
    cerr << "Dataset: creating keyframes ..." << endl;
    dataset.CreateKeyFrame();
    cerr << "Dataset: creating measurements ..." << endl;
    dataset.CreateMarkMeasure();    
    cerr << "Dataset: dataset created!" << endl << endl;

    //! Init solver
    cerr << "Solver: init solver ..." << endl;
    Solver solver(&dataset);

    //! Do calibrate with "initmk" algorithm
    cerr << "Solver: calibrate by initmk ..." << endl;
    solver.CalibInitMk(dataset.GetMsrMk(), dataset.GetMsrOdo());
    Se3 se3bc_initmk = solver.GetResult();
    cerr << "Solver: calibration done! " << se3bc_initmk << endl << endl;

    //! Do calibrate with "optmk" algorithm
    cerr << "Solver: calibrate by optmk ..." << endl;
    dataset.InitAll(solver.GetResult());
    solver.CalibOptMk(dataset.GetMsrMk(), dataset.GetMsrOdo());
    Se3 se3bc_optmk = solver.GetResult();
    cerr << "Solver: calibration done! " << se3bc_optmk << endl << endl;

    //! Debug: Show something here, with ros viewer
    MapPublish mappublish(&dataset);
    mappublish.run();

    return 0;
}
