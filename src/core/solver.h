#ifndef SOLVER_H
#define SOLVER_H

#include "dataset.h"
#include "measure.h"
#include "frame.h"
#include "type.h"
#include "orb/ORBmatcher.h"


namespace calibcamodo {

//!
//! \brief The Solver class
//! base class for all solvers
//!
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

//!
//! \brief The SolverAruco class
//! base class for solver using aruco mark measurements
//!
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

//!
//! \brief The SolverInitmk class
//! solver with algorith initmk
//! based on aruco mark measurements
//!
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


//!
//! \brief The SolverOptMk class
//! JointOptMk: using 3D translational mark measurements, iterative optimize SLAM and calibration
class SolverOptMk : public SolverAruco {
public:
    SolverOptMk(DatasetAruco *_pDataset);
    void DoCalib();
};



//!
//! \brief The SolverOrb class
//! solver using orb features
//!
class SolverOrb : public Solver {
public:
    SolverOrb(DatasetOrb* _pDataset);

    void DoCalib() {}
    void CreateMapPoint();

    // functions: find good matches of orb-features between 2 keyframes
    void MatchKeyPointOrb(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match);
    void RejectOutlierRansac(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                        const std::map<int, int>& match_in, std::map<int, int>& match_out);
    void RejectOutlierDist(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                        const std::map<int, int>& match_in, std::map<int, int>& match_out);

    // functions: create mappoints locally
    void CreateMapPointLocal(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                             const std::map<int, int>& match);
    void TriangulateMapPoint();

    // functions: detect loop closing in global
    void DetectLoopClose(PtrKeyFrameOrb pKf, std::vector<PtrKeyFrameOrb>& vecpKfCand);
    void MergeMapPointLoopClose();

    // functions: solve global optimization for slam or sclam
    void OptimizeSlam();
    void OptimizeCalibSlam();

private:
    DatasetOrb* mpDatasetOrb;
    ORBmatcher mOrbMatcher;

    // debug functions:
    void DrawMatches(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match, std::string imgtitle = "debug");
    cv::Mat ComputeCamMatP(PtrKeyFrame pKf, cv::Mat matCam);
};


}
#endif // SOLVER_H
