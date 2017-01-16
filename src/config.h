#ifndef CONFIG_H
#define CONFIG_H

namespace calibcamodo {

struct Config {
    static int NUM_FRAME;
    static double MARK_SIZE;
    static std::string STR_FOLDERPATH_MAIN;
    static double DATASET_THRESH_KF_ODOLIN;
    static double DATASET_THRESH_KF_ODOROT;
    static int ARUCO;
};

}
#endif
