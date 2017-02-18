#ifndef MAKERONLINE_BASE_H
#define MAKERONLINE_BASE_H

#include "dataset.h"

namespace calibcamodo {

class MakerOnlineBase
{
public:
    MakerOnlineBase(Dataset* _pDataset);

    virtual bool DoMakeOnce() = 0;

    bool RenewKfNow();
    void MakeMsrOdoNow();
    void InitKfNowPose();
    void InitKfLastPoseCov();

protected:
    Dataset* mpDataset;
    double mOdoLinErrR;
    double mOdoLinErrMin;
    double mOdoRotErrR;
    double mOdoRotErrRLin;
    double mOdoRotErrMin;
};

}

#endif // MAKERONLINE_BASE_H
