#ifndef MAPMARK_H
#define MAPMARK_H

#include "type.h"
#include <opencv2/core/core.hpp>

namespace calibcamodo {

class MapMark {
public:
    MapMark() = default;
    ~MapMark() = default;
    MapMark(int _id);
    MapMark(const MapMark &_mk);

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

class MapMarkAruco : public MapMark {
public:
    MapMarkAruco() = default;
    ~MapMarkAruco() = default;
    MapMarkAruco(int _id, int _arucoId, double _markSize);
    MapMarkAruco(const MapMark &_mk);
    MapMarkAruco(const MapMarkAruco &_mk);

protected:
    int mArucoId;
    double mMarkSize;
};

typedef std::shared_ptr<MapMark> PtrMapMark;
typedef std::shared_ptr<MapMarkAruco> PtrMapMarkAruco;

}
#endif
