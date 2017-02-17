#ifndef MAKER_ARUCO_H
#define MAKER_ARUCO_H

#include "maker_base.h"

namespace calibcamodo {

class MakerAruco : public MakerBase {

public:
    MakerAruco(DatasetAruco* _pDatasetAruco);

    void MakeMkAndMsrMk();
    void InitMkPose();
    void InitKfMkPose();

    virtual void DoMake() {
        MakeMsrOdo();
        InitKfPose();
        MakeMkAndMsrMk();
        InitMkPose();
    }

private:
    DatasetAruco* mpDatasetAruco;

    double mAmkZErrRZ;
    double mAmkZErrMin;
    double mAmkXYErrRZ;
    double mAmkXYErrMin;
};

}



# endif
