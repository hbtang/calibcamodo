#ifndef CONFIG_H
#define CONFIG_H

#include "stdafx.h"

namespace calibcamodo {

using namespace std;
using namespace cv;

struct Config {

    static int NUM_FRAME;
    static double MARK_SIZE;
    static string STR_FOLDERPATH_MAIN;

    static double DATASET_THRESH_KF_ODOLIN;
    static double DATASET_THRESH_KF_ODOROT;

    static int ARUCO_;

};

}
#endif
