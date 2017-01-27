#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <opencv/cv.h>

namespace calibcamodo {

class Config {
public:

    static void InitConfig(std::string _strfolderpathmain);

    //! Input
    static int NUM_FRAME;    
    static std::string STR_FOLDERPATH_MAIN;
    static std::string STR_FOlDERPATH_IMG;
    static std::string STR_FILEPATH_ODO;
    static std::string STR_FILEPATH_CAM;
    static std::string STR_FILEPATH_CALIB;
    static std::string STR_FILEPATH_ORBVOC;

    //!Camera Intrinsics
    static int IMAGE_WIDTH;
    static int IMAGE_HEIGHT;
    static cv::Mat CAMERA_MATRIX;
    static cv::Mat DISTORTION_COEFFICIENTS;

    //!Camera Extrinsics Init
    static cv::Mat RVEC_BC;
    static cv::Mat TVEC_BC;

    //! Mark
    static double MARK_SIZE;

    //! Dataset
    static double DATASET_THRESH_KF_ODOLIN;
    static double DATASET_THRESH_KF_ODOROT;    

    //! Solver
    static double CALIB_ODOLIN_ERRR;
    static double CALIB_ODOLIN_ERRMIN;
    static double CALIB_ODOROT_ERRR;
    static double CALIB_ODOROT_ERRRLIN;
    static double CALIB_ODOROT_ERRMIN;

    static double CALIB_AMKZ_ERRRZ;
    static double CALIB_AMKZ_ERRMIN;
    static double CALIB_AMKXY_ERRRZ;
    static double CALIB_AMKXY_ERRMIN;




    //! Output using ROS
    static double MAPPUB_SCALE_RATIO;
};

}
#endif
