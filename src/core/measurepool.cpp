#include "measurepool.h"

using namespace std;

namespace calibcamodo {

bool MeasurePoolSe2Kf2Kf::AddMsr(PtrMsrSe2Kf2Kf _pMsr) {
    PtrKeyFrame pKfHead = _pMsr->pKfHead;
    PtrKeyFrame pKfTail = _pMsr->pKfTail;

    if(msetMsr.count(_pMsr) != 0)
        return false;

    msetMsr.insert(_pMsr);
    mmapKfHead2Msr.emplace(pKfHead, _pMsr);
    mmapKfTail2Msr.emplace(pKfTail, _pMsr);
    return true;
}

void MeasurePoolSe2Kf2Kf::ClearAll() {
    msetMsr.clear();
    mmapKfHead2Msr.clear();
    mmapKfTail2Msr.clear();
}

PtrMsrSe2Kf2Kf MeasurePoolSe2Kf2Kf::GetMsrOdoByKfHead(PtrKeyFrame _pKf) const {
    PtrMsrSe2Kf2Kf pRet = nullptr;
    if(mmapKfHead2Msr.count(_pKf))
        pRet = mmapKfHead2Msr.at(_pKf);
    return pRet;
}

PtrMsrSe2Kf2Kf MeasurePoolSe2Kf2Kf::GetMsrOdoByKfTail(PtrKeyFrame _pKf) const {
    PtrMsrSe2Kf2Kf pRet = nullptr;
    if(mmapKfTail2Msr.count(_pKf))
        pRet = mmapKfTail2Msr.at(_pKf);
    return pRet;
}

PtrKeyFrame MeasurePoolSe2Kf2Kf::GetKfOdoNext(PtrKeyFrame _pKf) const {
    PtrMsrSe2Kf2Kf pMsr = GetMsrOdoByKfHead(_pKf);
    return pMsr ? pMsr->pKfTail : nullptr;
}


PtrKeyFrame MeasurePoolSe2Kf2Kf::GetKfOdoLast(PtrKeyFrame _pKf) const {
    PtrMsrSe2Kf2Kf pMsr = GetMsrOdoByKfTail(_pKf);
    return pMsr ? pMsr->pKfTail : nullptr;
}

}

//! MeasurePoolPt3Kf2Mk
namespace calibcamodo {

bool MeasurePoolPt3Kf2Mk::AddMsr(PtrMsrPt3Kf2Mk _pMsr) {
    PtrKeyFrame pKf = _pMsr->pKf;
    PtrMapMark pMk = _pMsr->pMk;

    if(msetMsr.count(_pMsr) != 0)
        return false;

    msetMsr.insert(_pMsr);
    mmapKf2Msr.emplace(pKf, _pMsr);
    mmapMk2Msr.emplace(pMk, _pMsr);
    return true;
}

std::set<PtrMsrPt3Kf2Mk> MeasurePoolPt3Kf2Mk::GetMsrByKf(PtrKeyFrame _pKf) const {
    set<PtrMsrPt3Kf2Mk> setpMsrRet;
    auto iter_lowerbound = mmapKf2Msr.lower_bound(_pKf);
    auto iter_upperbound = mmapKf2Msr.upper_bound(_pKf);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        setpMsrRet.insert((*iter).second);
    }
    return setpMsrRet;
}

std::set<PtrMsrPt3Kf2Mk> MeasurePoolPt3Kf2Mk::GetMsrByMk(PtrMapMark _pMk) const {
    set<PtrMsrPt3Kf2Mk> setpMsrRet;
    auto iter_lowerbound = mmapMk2Msr.lower_bound(_pMk);
    auto iter_upperbound = mmapMk2Msr.upper_bound(_pMk);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        setpMsrRet.insert((*iter).second);
    }
    return setpMsrRet;
}

PtrMsrPt3Kf2Mk MeasurePoolPt3Kf2Mk::GetMsrByKfMk(PtrKeyFrame _pKf, PtrMapMark _pMk) const {
    auto iter_lowerbound = mmapMk2Msr.lower_bound(_pMk);
    auto iter_upperbound = mmapMk2Msr.upper_bound(_pMk);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        PtrMsrPt3Kf2Mk pMsr = (*iter).second;
        if(pMsr->pKf == _pKf)
            return pMsr;
    }
    return nullptr;
}

std::set<PtrMapMark> MeasurePoolPt3Kf2Mk::GetMkByKf(PtrKeyFrame _pKf) const {
    set<PtrMsrPt3Kf2Mk> setMsr = GetMsrByKf(_pKf);
    set<PtrMapMark> setMkRet;
    for (auto pMsr : setMsr) {
        setMkRet.insert(static_cast<PtrMsrPt3Kf2Mk>(pMsr)->pMk);
    }
    return setMkRet;
}

std::set<PtrKeyFrame> MeasurePoolPt3Kf2Mk::GetKfByMk(PtrMapMark _pMk) const {
    set<PtrMsrPt3Kf2Mk> setMsr = GetMsrByMk(_pMk);
    set<PtrKeyFrame> setKfRet;
    for (auto pMsr : setMsr) {
        setKfRet.insert(static_cast<PtrMsrPt3Kf2Mk>(pMsr)->pKf);
    }
    return setKfRet;
}

}


//! MeasurePoolUVKf2Mp
//!
namespace calibcamodo {

bool MeasurePoolUVKf2Mp::AddMsr(PtrMsrUVKf2Mp _pMsr) {
    PtrKeyFrame pKf = _pMsr->pKf;
    PtrMapPoint pMp = _pMsr->pMp;
    int idKp = _pMsr->idKp;
    pair<PtrKeyFrame, int> pairKfId(pKf, idKp);
    pair<PtrKeyFrame, PtrMapPoint> pairKfMp(pKf, pMp);

//    assert(msetMsr.count(_pMsr) == 0);
//    assert(mmapKfId2Msr.count(pairKfId) == 0);
//    assert(mmapKfMp2Msr.count(pairKfMp) == 0);

    if(msetMsr.count(_pMsr) != 0)
        return false;
    if(mmapKfId2Msr.count(pairKfId) != 0)
        return false;
    if(mmapKfMp2Msr.count(pairKfMp) != 0)
        return false;

    msetMsr.insert(_pMsr);
    mmapKfId2Msr.emplace(pairKfId, _pMsr);
    mmapKfMp2Msr.emplace(pairKfMp, _pMsr);
    mmapKf2Msr.emplace(pKf, _pMsr);
    mmapMp2Msr.emplace(pMp, _pMsr);
    return true;
}

std::set<PtrMsrUVKf2Mp> MeasurePoolUVKf2Mp::GetMsrByKf(PtrKeyFrame _pKf) const {
    set<PtrMsrUVKf2Mp> setpMsrRet;
    auto iter_lowerbound = mmapKf2Msr.lower_bound(_pKf);
    auto iter_upperbound = mmapKf2Msr.upper_bound(_pKf);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        setpMsrRet.insert((*iter).second);
    }
    return setpMsrRet;
}

std::set<PtrMsrUVKf2Mp> MeasurePoolUVKf2Mp::GetMsrByMp(PtrMapPoint _pMp) const {
    set<PtrMsrUVKf2Mp> setpMsrRet;
    auto iter_lowerbound = mmapMp2Msr.lower_bound(_pMp);
    auto iter_upperbound = mmapMp2Msr.upper_bound(_pMp);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        setpMsrRet.insert((*iter).second);
    }
    return setpMsrRet;
}

PtrMsrUVKf2Mp MeasurePoolUVKf2Mp::GetMsrByKfId(PtrKeyFrame _pKf, int _idKp) const {
    std::pair<PtrKeyFrame, int> pairKfId(_pKf, _idKp);
    auto iter = mmapKfId2Msr.find(pairKfId);
    if (iter == mmapKfId2Msr.cend())
        return nullptr;
    else
        return iter->second;
}

PtrMsrUVKf2Mp MeasurePoolUVKf2Mp::GetMsrByKfMp(PtrKeyFrame _pKf, PtrMapPoint _pMp) const {
    std::pair<PtrKeyFrame, PtrMapPoint> pairKfMp(_pKf, _pMp);
    auto iter = mmapKfMp2Msr.find(pairKfMp);
    if (iter == mmapKfMp2Msr.cend())
        return nullptr;
    else
        return iter->second;
}

std::set<PtrKeyFrame> MeasurePoolUVKf2Mp::GetKfByMp(PtrMapPoint _pMp) const {
    set<PtrMsrUVKf2Mp> setMsr = GetMsrByMp(_pMp);
    set<PtrKeyFrame> setKfRet;
    for (auto pMsr : setMsr) {
        setKfRet.insert(static_cast<PtrMsrUVKf2Mp>(pMsr)->pKf);
    }
    return setKfRet;
}

std::set<PtrMapPoint> MeasurePoolUVKf2Mp::GetMpByKf(PtrKeyFrame _pKf) const {
    set<PtrMsrUVKf2Mp> setMsr = GetMsrByKf(_pKf);
    set<PtrMapPoint> setMpRet;
    for (auto pMsr : setMsr) {
        setMpRet.insert(static_cast<PtrMsrUVKf2Mp>(pMsr)->pMp);
    }
    return setMpRet;
}

PtrMapPoint MeasurePoolUVKf2Mp::GetMpByKfId(PtrKeyFrame _pKf, int _idKp) const {
    PtrMsrUVKf2Mp pMsr = GetMsrByKfId(_pKf, _idKp);
    return pMsr ? pMsr->pMp : nullptr;
}

}


