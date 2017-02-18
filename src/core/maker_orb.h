#ifndef MAKER_ORB_H
#define MAKER_ORB_H

#include "maker_base.h"
#include "orb/ORBmatcher.h"

namespace calibcamodo {

class MakerOrb : public MakerBase {

public:
    MakerOrb(DatasetOrb* _pDatasetOrb);

    virtual void DoMake() {
        MakeMsrOdo();
        InitKfPose();
        MakeMpAndMsrMp();
    }

    void MakeMpAndMsrMp();

    // functions: find good matches of orb-features between 2 keyframes
//    void MatchKeyPointOrb(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match);
//    void RejectOutlierRansac(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
//                        const std::map<int, int>& match_in, std::map<int, int>& match_out);
//    void RejectOutlierDist(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
//                        const std::map<int, int>& match_in, std::map<int, int>& match_out);

    // functions: init mappoints by triangulation, create mappoints and measure in dataset
    void InitMpByTrain(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                             const std::map<int, int>& match);
//    cv::Mat ComputeCamMatP(PtrKeyFrame pKf, const cv::Mat matCam);

    // functions: detect loop closing in global
    void DetectLoopClose(PtrKeyFrameOrb pKf, std::vector<PtrKeyFrameOrb>& vecpKfCand);
    void MergeMapPointLoopClose();

    // debug functions:
//    void DrawMatches(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match, std::string imgtitle = "debug");

protected:
    DatasetOrb* mpDatasetOrb;
    ORBmatcher mOrbMatcher;

};


}


#endif // MAKER_ORB_H
