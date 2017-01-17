#ifndef MARK_H
#define MARK_H

#include "type.h"
#include <opencv2/core/core.hpp>

namespace calibcamodo {

class Mark {
public:
    Mark();
    Mark(int _id);
    Mark(const Mark &_mk);
    ~Mark() {}

    inline int GetId() const {return mId;}
    inline Se3 GetPose() const {return mSe3wm;}
    inline void SetPose(Se3 _in) { mSe3wm = _in; }
    void SetPoseTranslation(cv::Mat _tvec);

protected:
    int mId;
    Se3 mSe3wm;
};

class ArucoMark : public Mark {
public:
    ArucoMark() : Mark() {}
    ArucoMark(const Mark &_mk) : Mark(_mk) {}
    ~ArucoMark() {}

    void InsertMsrMk(PtrMsrKf2AMk pmsr);
    void DeleteMsrMk(PtrMsrKf2AMk pmsr);    

    std::set<PtrMsrKf2AMk> GetMsr() const {return msetpMsr;}
    std::set<PtrMsrKf2AMk> GetMsr(std::set<PtrKeyFrame> _setpKf) const; // return measure from given kfs
    std::set<PtrKeyFrame> GetKf() const {return msetpKf;}

protected:

    std::set<PtrMsrKf2AMk> msetpMsr;
    std::set<PtrKeyFrame> msetpKf;
    std::map<PtrKeyFrame, PtrMsrKf2AMk> mmappKf2pMsr;
};

}
#endif
