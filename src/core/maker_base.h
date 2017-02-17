#ifndef MAKER_H
#define MAKER_H

#include "dataset.h"

namespace calibcamodo {

class MakerBase
{
public:
    MakerBase(Dataset* _pDataset);

    void MakeMsrOdo();
    void InitKfPose();

    virtual void DoMake() = 0;

protected:
    Dataset *mpDataset;

    double mOdoLinErrR;
    double mOdoLinErrMin;
    double mOdoRotErrR;
    double mOdoRotErrRLin;
    double mOdoRotErrMin;
};

}
#endif // MAKER_H
