#ifndef MEASUREPOOL_H
#define MEASUREPOOL_H

#include "measure.h"

namespace calibcamodo {

class MeasurePoolOrbMp
{
public:
    MeasurePoolOrbMp() = default;
    bool AddMsr(PtrMsrUVKf2Mp _pMsr);
//    bool DelMsr(PtrMsrUVKf2Mp _pMsr);

    std::set<PtrMsrUVKf2Mp> GetMsrAll() { return msetMsrMp; }
    std::set<PtrMsrUVKf2Mp> GetMsrByKf(PtrKeyFrame _pKf);
    std::set<PtrMsrUVKf2Mp> GetMsrByMp(PtrMapPoint _pMp);
    PtrMsrUVKf2Mp GetMsrByKfId(PtrKeyFrame _pKf, int _idKp);
    PtrMsrUVKf2Mp GetMsrByKfMp(PtrKeyFrame _pKf, PtrMapPoint _pMp);
    std::set<PtrKeyFrame> GetKfByMp(PtrMapPoint _pMp);
    std::set<PtrMapPoint> GetMpByKf(PtrKeyFrame _pKf);

private:
    std::set<PtrMsrUVKf2Mp> msetMsrMp;
    std::map<std::pair<PtrKeyFrame, int>, PtrMsrUVKf2Mp> mmapKfId2Msr;
    std::map<std::pair<PtrKeyFrame, PtrMapPoint>, PtrMsrUVKf2Mp> mmapKfMp2Msr;
    std::multimap<PtrKeyFrame, PtrMsrUVKf2Mp> mmapKf2Msr;
    std::multimap<PtrMapPoint, PtrMsrUVKf2Mp> mmapMp2Msr;
};

}



#endif // MEASUREPOOL_H
