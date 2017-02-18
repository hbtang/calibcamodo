#ifndef MAPMARK_H
#define MAPMARK_H

#include "type.h"
#include <opencv2/core/core.hpp>

namespace calibcamodo {

class Mark {
public:
    Mark() = default;
    ~Mark() = default;
    Mark(int _id);
    Mark(const Mark &_mk);

    inline void SetId(int _id) { mId = _id; }
    inline int GetId() const { return mId; }

    inline void SetPose(Se3 _in) { mSe3wm = _in; }
    inline void SetPoseTvec(cv::Mat _tvec) {
        mSe3wm.tvec = _tvec.clone();
    }
    inline void SetPoseRvec(cv::Mat _rvec) {
        mSe3wm.rvec = _rvec.clone();
    }
    inline Se3 GetPose() const { return mSe3wm; }

protected:
    int mId;
    Se3 mSe3wm;
};

class MarkAruco : public Mark {
public:
    MarkAruco() = default;
    ~MarkAruco() = default;
    MarkAruco(int _id, int _arucoId, double _markSize);
    MarkAruco(const Mark &_mk);
    MarkAruco(const MarkAruco &_mk);

protected:
    int mArucoId;
    double mMarkSize;
};

typedef std::shared_ptr<Mark> PtrMark;
typedef std::shared_ptr<MarkAruco> PtrMarkAruco;

}
#endif
