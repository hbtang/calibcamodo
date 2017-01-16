#include "mark.h"
#include "frame.h"

namespace calibcamodo {

using namespace std;
using namespace cv;


Mark::Mark() : mId(-1) {}

Mark::Mark(int _id) : mId(_id) {}

Mark::Mark(const Mark &_mk) : mId(_mk.mId), mSe3wm(_mk.mSe3wm) {}

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

void Mark::SetPoseTranslation(Mat _tvec) {
    mSe3wm.tvec = _tvec.clone();
}

}
