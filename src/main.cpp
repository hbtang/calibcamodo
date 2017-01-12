
#include "aruco.h"
#include "cvdrawingutils.h"
#include "stdafx.h"

#include "Dataset.h"
#include "Frame.h"
#include "Measure.h"
#include "Mark.h"
#include "Solver.h"

using namespace std;
using namespace cv;
using namespace aruco;
using namespace calibcamodo;

int main(int argc, char **argv) {

//    Mat m1 = (Mat_<float>(2,2)  << 1, 2, 3, 4);
//    Mat m2 = fun(m1);
//    m2.at<float>(0,0) = 5;
//    cerr << m1 << endl << m2 << endl;

    // INIT ROS
    ros::init(argc, argv, "pub");
    ros::start();
    ros::NodeHandle nh;
    ros::Rate rate(100);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw",1);

    // INIT DATASET
    string strFolderPathMain = argv[1];
    int numFrame = atoi(argv[2]);
    double markerSize = atof(argv[3]);

    Dataset dataset(strFolderPathMain, numFrame, markerSize);
    dataset.CreateFrame();
    dataset.CreateKeyFrame();
    dataset.CreateMarkMeasure();

    // INIT SOLVER
    Solver solver;

    // DO CALIBRATE
    solver.CalibInitMk(dataset.GetMsrMk(), dataset.GetMsrOdo());



    // DEBUG: SHOW SOMETHING
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
