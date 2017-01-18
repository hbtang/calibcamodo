#ifndef CONFIG_H
#define CONFIG_H

#include <string>

namespace calibcamodo {

class Config {
public:

    static void InitConfig(std::string _strfolderpathmain, int numframe, double marksize);

    //! Read dataset
    static int NUM_FRAME;    
    static std::string STR_FOLDERPATH_MAIN;
    static std::string STR_FOlDERPATH_IMG;
    static std::string STR_FILEPATH_ODO;
    static std::string STR_FILEPATH_CAM;
    static std::string STR_FILEPATH_CALIB;

    //! Dataset
    static double DATASET_THRESH_KF_ODOLIN;
    static double DATASET_THRESH_KF_ODOROT;
    static double MARK_SIZE;

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
