#ifndef MARK_H
#define MARK_H

#include "Type.h"
#include "stdafx.h"

namespace calibcamodo {

using namespace std;
using namespace cv;

class Mark {
public:
    Mark();
    Mark(int _id);
    Mark(const Mark &_mk);
    ~Mark() {}

    int GetId() const {return mId;}

protected:
    int mId;
    Mat mrvec_wm;
    Mat mtvec_wm;
};

class ArucoMark : public Mark {
public:
    ArucoMark() : Mark() {}
    ArucoMark(const Mark &_mk) : Mark(_mk) {}
    ~ArucoMark() {}

    void InsertMsrMk(PtrMsrKf2AMk pmsr);
    void DeleteMsrMk(PtrMsrKf2AMk pmsr);    

    set<PtrMsrKf2AMk> GetMsr() const {return msetpMsr;}
    set<PtrMsrKf2AMk> GetMsr(set<PtrKeyFrame> _setpKf) const; // return measure from given kfs
    set<PtrKeyFrame> GetKf() const {return msetpKf;}

protected:

    set<PtrMsrKf2AMk> msetpMsr;
    set<PtrKeyFrame> msetpKf;
    map<PtrKeyFrame, PtrMsrKf2AMk> mmappKf2pMsr;
};

}
#endif
