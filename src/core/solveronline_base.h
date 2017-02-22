#ifndef SOLVERONLINE_BASE_H
#define SOLVERONLINE_BASE_H

#include "dataset.h"

namespace calibcamodo {

class SolverOnlineBase {

public:
    SolverOnlineBase(Dataset* _pDataset);
    virtual void DoFilterOnce() = 0;

protected:
    Dataset* mpDataset;

};


}


#endif // SOLVERONLINE_BASE_H
