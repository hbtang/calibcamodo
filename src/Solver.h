#ifndef SOLVER_H
#define SOLVER_H

#include "Measure.h"
#include "Frame.h"
#include "Type.h"
#include "stdafx.h"

namespace calibcamodo {

using namespace std;
using namespace cv;

struct HyperEdgeOdoMk {

    HyperEdgeOdoMk(PtrMsrSe2Kf2Kf _pMsrOdo, PtrMsrKf2AMk _pMsrMk1, PtrMsrKf2AMk _pMsrMk2):
        pMsrOdo(_pMsrOdo), pMsrMk1(_pMsrMk1), pMsrMk2(_pMsrMk2) {}

    PtrMsrSe2Kf2Kf pMsrOdo;
    PtrMsrKf2AMk pMsrMk1;
    PtrMsrKf2AMk pMsrMk2;
};

class Solver {
public:
    Solver();
    ~Solver() {}

    // InitMk: using 3D translational mark measurements, non-iterative
    void CalibInitMk(const set<PtrMsrKf2AMk> &_measuremk, const set<PtrMsrSe2Kf2Kf> &_measureodo);
    void ComputeGrndPlane(const set<PtrMsrKf2AMk> &_measure, Mat &nvec_cg);
    void ComputeCamProjFrame(const Mat &nvec_cg, Mat &rvec_dc, Mat &tvec_dc, int flag = 0);
    double Compute2DExtrinsic(const set<PtrMsrKf2AMk> &_measuremk, const set<PtrMsrSe2Kf2Kf> &_measureodo,
                            const Mat &rvec_dc, const Mat &tvec_dc, Mat &rvec_bd, Mat &tvec_bd);

    // JointOptMk: using 3D translational mark measurements, iterative optimize SLAM and calibration
    void CalibOptMk(const set<PtrMsrKf2AMk> &_measuremk, const set<PtrMsrSe2Kf2Kf> &_measureodo);


    // other functions ...
    int FindCovisMark(const PtrKeyFrame _pKf1, const PtrKeyFrame _pKf2, set<pair<PtrMsrKf2AMk, PtrMsrKf2AMk>> &_setpairMsr);
    void GetResult(Mat &rvec_bc, Mat &tvec_bc) const {
        mrvec_bc.copyTo(rvec_bc);
        mtvec_bc.copyTo(tvec_bc);
    }

private:

    Mat mrvec_bc;
    Mat mtvec_bc;

};

}
#endif // SOLVER_H
