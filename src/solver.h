#ifndef SOLVER_H
#define SOLVER_H

#include "measure.h"
#include "frame.h"
#include "type.h"

namespace calibcamodo {

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
    void ComputeGrndPlane(const set<PtrMsrKf2AMk> &_measure, cv::Mat &nvec_cg);
    void ComputeCamProjFrame(const cv::Mat &nvec_cg, cv::Mat &rvec_dc, cv::Mat &tvec_dc, int flag = 0);
    double Compute2DExtrinsic(const set<PtrMsrKf2AMk> &_measuremk, const set<PtrMsrSe2Kf2Kf> &_measureodo,
                            const cv::Mat &rvec_dc, const cv::Mat &tvec_dc, cv::Mat &rvec_bd, cv::Mat &tvec_bd);

    // JointOptMk: using 3D translational mark measurements, iterative optimize SLAM and calibration
    void CalibOptMk(const set<PtrMsrKf2AMk> &_measuremk, const set<PtrMsrSe2Kf2Kf> &_measureodo);


    // other functions ...
    int FindCovisMark(const PtrKeyFrame _pKf1, const PtrKeyFrame _pKf2, set<pair<PtrMsrKf2AMk, PtrMsrKf2AMk>> &_setpairMsr);
    void GetResult(cv::Mat &rvec_bc, cv::Mat &tvec_bc) const {
        mSe3cb.rvec.copyTo(rvec_bc);
        mSe3cb.tvec.copyTo(tvec_bc);
    }

private:
    Se3 mSe3cb;



};

}
#endif // SOLVER_H
