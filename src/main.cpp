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

using namespace std;
using namespace cv;
using namespace aruco;
using namespace calibcamodo;

int main(int argc, char **argv) {

    string strFolderPathMain = argv[1];
    int numFrame = atoi(argv[2]);
    double markerSize = atof(argv[3]);

    //! Init ros

    //! Init config
    Config::InitConfig(strFolderPathMain, numFrame, markerSize);

    //! Init dataset
    Dataset dataset;
    dataset.CreateFrame();
    dataset.CreateKeyFrame();
    dataset.CreateMarkMeasure();    

    //! Init solver
    Solver solver(&dataset);

    //! Do calibrate with "initmk" algorithm
    solver.CalibInitMk(dataset.GetMsrMk(), dataset.GetMsrOdo());
    Se3 se3bc_initmk = solver.GetResult();
    cerr << "initmk se3bc: " << se3bc_initmk << endl;

    //! Do calibrate with "optmk" algorithm
    dataset.InitAll(solver.GetResult());
    solver.CalibOptMk(dataset.GetMsrMk(), dataset.GetMsrOdo());
    Se3 se3bc_optmk = solver.GetResult();
    cerr << "optmk se3bc: " << se3bc_optmk << endl;


    return 0;
}
