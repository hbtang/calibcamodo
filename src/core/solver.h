#ifndef SOLVER_H
#define SOLVER_H

#include "dataset.h"
#include "measure.h"
#include "frame.h"
#include "type.h"
#include "orb/ORBmatcher.h"
#include "g2o/g2o_api.h"


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

    //!
    //! \brief CreateMapPoint:
    //! create mappoints from 2 consecutive keyframes
    //! reject outliers by ransac and distance
    //! init mappoint location by linear triangulation
    //! reject bad parallex point
    //!
    void CreateMapPoints();

    // functions: find good matches of orb-features between 2 keyframes
    void MatchKeyPointOrb(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match);
    void RejectOutlierRansac(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                        const std::map<int, int>& match_in, std::map<int, int>& match_out);
    void RejectOutlierDist(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                        const std::map<int, int>& match_in, std::map<int, int>& match_out);

    // functions: init mappoints by triangulation, create mappoints and measure in dataset
    void InitMapPointTrian(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                             const std::map<int, int>& match);

    // functions: detect loop closing in global
    void DetectLoopClose(PtrKeyFrameOrb pKf, std::vector<PtrKeyFrameOrb>& vecpKfCand);
    void MergeMapPointLoopClose();

    // functions: solve global optimization for slam or sclam
    void OptimizeSlam();
    void OptimizeSclam();

private:
    DatasetOrb* mpDatasetOrb;
    ORBmatcher mOrbMatcher;

    // low level functions
    cv::Mat ComputeCamMatP(PtrKeyFrame pKf, cv::Mat matCam);

    // debug functions
    void DrawMatches(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match, std::string imgtitle = "debug");
    void PrintEdgeInfoOdo(const std::vector<g2o::EdgeSE2*>& vecpEdgeOdo);
    void PrintEdgeInfoVSlam(const std::vector<g2o::EdgeVSlam*>& vecpEdgeVSlam);
};


}
#endif // SOLVER_H
