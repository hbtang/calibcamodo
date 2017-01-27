#ifndef SOLVER_H
#define SOLVER_H

#include "dataset.h"
#include "measure.h"
#include "frame.h"
#include "type.h"

namespace calibcamodo {

class Solver {
public:
    Solver(Dataset *_pDataset);

    void CreateMsrOdos();
    void RefreshKfsPose();

    inline Se3 GetSe3cb() const { return mSe3cb; }
    inline void SetSe3cb(Se3 _se3) { mSe3cb = _se3; }
    virtual void DoCalib() = 0;

protected:
    Se3 mSe3cb;
    Dataset *mpDataset;

    double mOdoLinErrR;
    double mOdoLinErrMin;
    double mOdoRotErrR;
    double mOdoRotErrRLin;
    double mOdoRotErrMin;
};

class SolverAruco : public Solver {
public:
    SolverAruco(DatasetAruco* _pDatasetAruco);
    void CreateMarks();
    void RefreshMksPose();
    void RefreshAllPose();

private:
    DatasetAruco* mpDatasetAruco;

    double mAmkZErrRZ;
    double mAmkZErrMin;
    double mAmkXYErrRZ;
    double mAmkXYErrMin;
};

class SolverInitmk : public SolverAruco {

public:
    SolverInitmk(DatasetAruco* _pDataset);

    struct HyperEdgeOdoMk {
        HyperEdgeOdoMk(PtrMsrSe2Kf2Kf _pMsrOdo, PtrMsrPt3Kf2Mk _pMsrMk1, PtrMsrPt3Kf2Mk _pMsrMk2):
            pMsrOdo(_pMsrOdo), pMsrMk1(_pMsrMk1), pMsrMk2(_pMsrMk2) {}
        PtrMsrSe2Kf2Kf pMsrOdo;
        PtrMsrPt3Kf2Mk pMsrMk1;
        PtrMsrPt3Kf2Mk pMsrMk2;
    };

    void DoCalib();
    void ComputeGrndPlane(cv::Mat &nvec_cg);
    void ComputeCamProjFrame(const cv::Mat &nvec_cg, cv::Mat &rvec_dc, cv::Mat &tvec_dc);
    double Compute2DExtrinsic(const cv::Mat &rvec_dc, const cv::Mat &tvec_dc, cv::Mat &rvec_bd, cv::Mat &tvec_bd);
    int FindCovisMark(const PtrKeyFrame _pKf1, const PtrKeyFrame _pKf2, set<pair<PtrMsrPt3Kf2Mk, PtrMsrPt3Kf2Mk>> &_setpairMsr);

private:
    std::set<HyperEdgeOdoMk> msetHyperEdge;
};


// JointOptMk: using 3D translational mark measurements, iterative optimize SLAM and calibration
class SolverOptMk : public SolverAruco {
public:
    SolverOptMk(DatasetAruco *_pDataset);
    void DoCalib();

};

}
#endif // SOLVER_H
