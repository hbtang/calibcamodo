#ifndef SOLVER_OPTMK_H
#define SOLVER_OPTMK_H

#include "solver_base.h"

namespace calibcamodo {

class SolverOptMk: public SolverBase
{
public:
    SolverOptMk(Dataset* _pDataset);
    virtual void DoCalib();
};

}
#endif // SOLVER_OPTMK_H
