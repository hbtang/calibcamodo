#ifndef SOLVER_INITMK_H
#define SOLVER_INITMK_H

#include "solver_base.h"

namespace calibcamodo {

class SolverInitMk: public SolverBase
{
public:
    SolverInitMk(Dataset* _pDataset);

    struct HyperEdgeOdoMk {
        HyperEdgeOdoMk(PtrMsrSe2Kf2Kf _pMsrOdo, PtrMsrPt3Kf2Mk _pMsrMk1, PtrMsrPt3Kf2Mk _pMsrMk2):
            pMsrOdo(_pMsrOdo), pMsrMk1(_pMsrMk1), pMsrMk2(_pMsrMk2) {}
        PtrMsrSe2Kf2Kf pMsrOdo;
        PtrMsrPt3Kf2Mk pMsrMk1;
        PtrMsrPt3Kf2Mk pMsrMk2;
    };

    virtual void DoCalib();
    void ComputeGrndPlane(cv::Mat &nvec_cg);
    void ComputeCamProjFrame(const cv::Mat &nvec_cg, cv::Mat &rvec_dc, cv::Mat &tvec_dc);
    double Compute2DExtrinsic(const cv::Mat &rvec_dc, const cv::Mat &tvec_dc, cv::Mat &rvec_bd, cv::Mat &tvec_bd);
    int FindCovisMark(const PtrKeyFrame _pKf1, const PtrKeyFrame _pKf2, set<pair<PtrMsrPt3Kf2Mk, PtrMsrPt3Kf2Mk>> &_setpairMsr);

private:
    std::set<HyperEdgeOdoMk> msetHyperEdge;
};

}
#endif // SOLVER_INITMK_H
