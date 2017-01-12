#include "Mark.h"
#include "Frame.h"

namespace calibcamodo {

using namespace std;
using namespace cv;


Mark::Mark() : mId(0) {
    mrvec_wm.create(3, 1, CV_32FC1);
    mtvec_wm.create(3, 1, CV_32FC1);
    for (int i=0;i<3;i++)
        mrvec_wm.at<float>(i,0)=mtvec_wm.at<float>(i,0)=-999999;
}

Mark::Mark(int _id) : mId(_id) {
    mrvec_wm.create(3, 1, CV_32FC1);
    mtvec_wm.create(3, 1, CV_32FC1);
    for (int i=0;i<3;i++)
        mrvec_wm.at<float>(i,0)=mtvec_wm.at<float>(i,0)=-999999;
}

Mark::Mark(const Mark &_mk) : mId(_mk.mId) {
    _mk.mrvec_wm.copyTo(mrvec_wm);
    _mk.mtvec_wm.copyTo(mtvec_wm);
}

void ArucoMark::InsertMsrMk(PtrMsrKf2AMk pmsr) {
    if(msetpKf.count(pmsr->pKf)) {
        cerr << "Error in ArucoMark::InsertMsrMk, already observed." << endl;
        return;
    }
    msetpMsr.insert(pmsr);
    msetpKf.insert(pmsr->pKf);
    mmappKf2pMsr[pmsr->pKf] = pmsr;
}

void ArucoMark::DeleteMsrMk(PtrMsrKf2AMk pmsr) {
    msetpMsr.erase(pmsr);
    msetpKf.erase(pmsr->pKf);
    mmappKf2pMsr.erase(pmsr->pKf);
}

set<PtrMsrKf2AMk> ArucoMark::GetMsr(set<PtrKeyFrame> _setpKf) const {
    set<PtrMsrKf2AMk> setRet;
    for (auto pKf : _setpKf) {
        if(msetpKf.count(pKf)) {
            PtrMsrKf2AMk pMsr = mmappKf2pMsr.at(pKf);
            setRet.insert(pMsr);
        }
    }
    return setRet;
}

}
