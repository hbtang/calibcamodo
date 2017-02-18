#ifndef MEASUREPOOL_H
#define MEASUREPOOL_H

#include "measure.h"

namespace calibcamodo {

//!
//! \brief The MeasurePoolSe2Kf2Kf class
//!
class MeasurePoolSe2Kf2Kf {
public:
    MeasurePoolSe2Kf2Kf() = default;

    bool AddMsr(PtrMsrSe2Kf2Kf _pMsr);
    void ClearAll();

    std::set<PtrMsrSe2Kf2Kf> GetMsrAll() const { return msetMsr; }
    PtrMsrSe2Kf2Kf GetMsrOdoByKfHead(PtrKeyFrame _pKf) const;
    PtrMsrSe2Kf2Kf GetMsrOdoByKfTail(PtrKeyFrame _pKf) const;
    PtrKeyFrame GetKfOdoNext(PtrKeyFrame _pKf) const;
    PtrKeyFrame GetKfOdoLast(PtrKeyFrame _pKf) const;

private:
    std::set<PtrMsrSe2Kf2Kf> msetMsr;
    std::map<PtrKeyFrame, PtrMsrSe2Kf2Kf> mmapKfHead2Msr;
    std::map<PtrKeyFrame, PtrMsrSe2Kf2Kf> mmapKfTail2Msr;
};

//!
//! \brief The MeasurePoolPt3Kf2Mk class
//!
class MeasurePoolPt3Kf2Mk {
public:
    MeasurePoolPt3Kf2Mk() = default;

    bool AddMsr(PtrMsrPt3Kf2Mk _pMsr);

    std::set<PtrMsrPt3Kf2Mk> GetMsrAll() const { return msetMsr; }
    std::set<PtrMsrPt3Kf2Mk> GetMsrByKf(PtrKeyFrame _pKf) const;
    std::set<PtrMsrPt3Kf2Mk> GetMsrByMk(PtrMapMark _pMk) const;
    PtrMsrPt3Kf2Mk GetMsrByKfMk(PtrKeyFrame _pKf, PtrMapMark _pMk) const;
    std::set<PtrMapMark> GetMkByKf(PtrKeyFrame _pKf) const;
    std::set<PtrKeyFrame> GetKfByMk(PtrMapMark _pMk) const;

private:
    std::set<PtrMsrPt3Kf2Mk> msetMsr;
    std::multimap<PtrKeyFrame, PtrMsrPt3Kf2Mk> mmapKf2Msr;
    std::multimap<PtrMapMark, PtrMsrPt3Kf2Mk> mmapMk2Msr;
};

//!
//! \brief The MeasurePoolUVKf2Mp class
//!
class MeasurePoolUVKf2Mp {
public:
    MeasurePoolUVKf2Mp() = default;

    bool AddMsr(PtrMsrUVKf2Mp _pMsr);
//    bool DelMsr(PtrMsrUVKf2Mp _pMsr);

    std::set<PtrMsrUVKf2Mp> GetMsrAll() const { return msetMsr; }
    std::set<PtrMsrUVKf2Mp> GetMsrByKf(PtrKeyFrame _pKf) const;
    std::set<PtrMsrUVKf2Mp> GetMsrByMp(PtrMapPoint _pMp) const;
    PtrMsrUVKf2Mp GetMsrByKfId(PtrKeyFrame _pKf, int _idKp) const;
    PtrMsrUVKf2Mp GetMsrByKfMp(PtrKeyFrame _pKf, PtrMapPoint _pMp) const;
    std::set<PtrKeyFrame> GetKfByMp(PtrMapPoint _pMp) const;
    std::set<PtrMapPoint> GetMpByKf(PtrKeyFrame _pKf) const;
    PtrMapPoint GetMpByKfId(PtrKeyFrame _pKf, int _idKp) const;

private:
    std::set<PtrMsrUVKf2Mp> msetMsr;
    std::map<std::pair<PtrKeyFrame, int>, PtrMsrUVKf2Mp> mmapKfId2Msr;
    std::map<std::pair<PtrKeyFrame, PtrMapPoint>, PtrMsrUVKf2Mp> mmapKfMp2Msr;
    std::multimap<PtrKeyFrame, PtrMsrUVKf2Mp> mmapKf2Msr;
    std::multimap<PtrMapPoint, PtrMsrUVKf2Mp> mmapMp2Msr;
};



}



#endif // MEASUREPOOL_H
