#ifndef SOLVERONLINE_EKF_H
#define SOLVERONLINE_EKF_H

#include "solveronline_base.h"

namespace calibcamodo {


class SolverOnlineEkf : public SolverOnlineBase {

public:
    SolverOnlineEkf(Dataset* _pDataset);
    virtual void DoFilterOnce();

    void Propagate(PtrMsrSe2Kf2Kf _pMsrOdo) const;
    void Correct(std::set<PtrMsrUVKf2Mp> _setpMsrMp) const;
};


}



#endif // SOLVERONLINE_EKF_H
