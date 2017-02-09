#include "measurepool.h"

using namespace std;

namespace calibcamodo {

bool MeasurePoolOrbMp::AddMsr(PtrMsrUVKf2Mp _pMsr) {
    PtrKeyFrame pKf = _pMsr->pKf;
    PtrMapPoint pMp = _pMsr->pMp;
    int idKp = _pMsr->idKp;
    pair<PtrKeyFrame, int> pairKfId(pKf, idKp);
    pair<PtrKeyFrame, PtrMapPoint> pairKfMp(pKf, pMp);

    assert(msetMsrMp.count(_pMsr) == 0);
    if(msetMsrMp.count(_pMsr) != 0){
        return false;
    }

    msetMsrMp.insert(_pMsr);
    mmapKfId2Msr.emplace(pairKfId, _pMsr);
    mmapKfMp2Msr.emplace(pairKfMp, _pMsr);
    mmapKf2Msr.emplace(pKf, _pMsr);
    mmapMp2Msr.emplace(pMp, _pMsr);
    return true;
}

std::set<PtrMsrUVKf2Mp> MeasurePoolOrbMp::GetMsrByKf(PtrKeyFrame _pKf) {
    set<PtrMsrUVKf2Mp> setpMsrRet;
    auto iter_lowerbound = mmapKf2Msr.lower_bound(_pKf);
    auto iter_upperbound = mmapKf2Msr.upper_bound(_pKf);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        setpMsrRet.insert((*iter).second);
    }
    return setpMsrRet;
}

std::set<PtrMsrUVKf2Mp> MeasurePoolOrbMp::GetMsrByMp(PtrMapPoint _pMp) {
    set<PtrMsrUVKf2Mp> setpMsrRet;
    auto iter_lowerbound = mmapMp2Msr.lower_bound(_pMp);
    auto iter_upperbound = mmapMp2Msr.upper_bound(_pMp);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        setpMsrRet.insert((*iter).second);
    }
    return setpMsrRet;
}

PtrMsrUVKf2Mp MeasurePoolOrbMp::GetMsrByKfId(PtrKeyFrame _pKf, int _idKp) {
    std::pair<PtrKeyFrame, int> pairKfId(_pKf, _idKp);
    auto iter = mmapKfId2Msr.find(pairKfId);
    if (iter == mmapKfId2Msr.cend())
        return nullptr;
    else
        return iter->second;
}

PtrMsrUVKf2Mp MeasurePoolOrbMp::GetMsrByKfMp(PtrKeyFrame _pKf, PtrMapPoint _pMp) {
    std::pair<PtrKeyFrame, PtrMapPoint> pairKfMp(_pKf, _pMp);
    auto iter = mmapKfMp2Msr.find(pairKfMp);
    if (iter == mmapKfMp2Msr.cend())
        return nullptr;
    else
        return iter->second;
}

std::set<PtrKeyFrame> MeasurePoolOrbMp::GetKfByMp(PtrMapPoint _pMp) {
    set<PtrMsrUVKf2Mp> setMsr = GetMsrByMp(_pMp);
    set<PtrKeyFrame> setKfRet;
    for (auto pMsr : setMsr) {
        setKfRet.insert(static_cast<PtrMsrUVKf2Mp>(pMsr)->pKf);
    }
    return setKfRet;
}

std::set<PtrMapPoint> MeasurePoolOrbMp::GetMpByKf(PtrKeyFrame _pKf) {
    set<PtrMsrUVKf2Mp> setMsr = GetMsrByKf(_pKf);
    set<PtrMapPoint> setMpRet;
    for (auto pMsr : setMsr) {
        setMpRet.insert(static_cast<PtrMsrUVKf2Mp>(pMsr)->pMp);
    }
    return setMpRet;
}

}


