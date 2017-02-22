#ifndef SOLVERBASE_H
#define SOLVERBASE_H

#include "dataset.h"


namespace calibcamodo {

class SolverBase
{
public:
    SolverBase(Dataset *_pDataset);
    virtual void DoCalib() = 0;

protected:
    Dataset* mpDataset;

};

}
#endif // SOLVERBASE_H
