#include "config.h"
#include "type.h"

namespace calibcamodo {

//! IO
int Config::NUM_FRAME;
std::string Config::STR_FOLDERPATH_MAIN;
std::string Config::STR_FOlDERPATH_IMG;
std::string Config::STR_FILEPATH_ODO;
std::string Config::STR_FILEPATH_CAM;
std::string Config::STR_FILEPATH_CALIB;

//! Dataset
double Config::DATASET_THRESH_KF_ODOLIN;
double Config::DATASET_THRESH_KF_ODOROT;
double Config::MARK_SIZE;

//! Solver
double Config::CALIB_ODOLIN_ERRR;
double Config::CALIB_ODOLIN_ERRMIN;
double Config::CALIB_ODOROT_ERRR;
double Config::CALIB_ODOROT_ERRRLIN;
double Config::CALIB_ODOROT_ERRMIN;

double Config::CALIB_AMKZ_ERRRZ;
double Config::CALIB_AMKZ_ERRMIN;
double Config::CALIB_AMKXY_ERRRZ;
double Config::CALIB_AMKXY_ERRMIN;

//! ROS PUBLISHER
double Config::MAPPUB_SCALE_RATIO;

void Config::InitConfig(std::string _strfolderpathmain,
                   int numframe, double marksize) {

    STR_FOLDERPATH_MAIN = _strfolderpathmain;
    STR_FOlDERPATH_IMG = _strfolderpathmain+"image/";
    STR_FILEPATH_ODO = _strfolderpathmain+"/rec/Odo.rec";
    STR_FILEPATH_CAM = _strfolderpathmain+"config/CamConfig.yml";
    NUM_FRAME = numframe;
    MARK_SIZE = marksize;

    DATASET_THRESH_KF_ODOLIN = 100;
    DATASET_THRESH_KF_ODOROT = 5*PI/180;

    CALIB_ODOLIN_ERRR = 0.01;
    CALIB_ODOLIN_ERRMIN = 1;
    CALIB_ODOROT_ERRR = 0.01;
    CALIB_ODOROT_ERRRLIN = 1*PI/180/10000;
    CALIB_ODOROT_ERRMIN = 0.1*PI/180;

    CALIB_AMKZ_ERRRZ = 0.05;
    CALIB_AMKZ_ERRMIN = 1;
    CALIB_AMKXY_ERRRZ = 0.01;
    CALIB_AMKXY_ERRMIN = 1;

    MAPPUB_SCALE_RATIO = 300;
}

}
