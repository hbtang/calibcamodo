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

    int GetId() const {return mId;}

protected:
    int mId;
    cv::Mat mrvec_wm;
    cv::Mat mtvec_wm;
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
