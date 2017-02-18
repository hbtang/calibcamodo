#ifndef MAKERONLINE_ORB_H
#define MAKERONLINE_ORB_H

#include "makeronline_base.h"

namespace calibcamodo {

class MakerOnlineOrb : public MakerOnlineBase
{
public:
    MakerOnlineOrb(DatasetOrb* _pDatasetOrb);

    virtual bool DoMakeOnce();

protected:
    DatasetOrb* mpDatasetOrb;
};

}
#endif // MAKERONLINE_ORB_H
