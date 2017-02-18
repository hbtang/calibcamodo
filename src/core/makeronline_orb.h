#ifndef MAKERONLINE_ORB_H
#define MAKERONLINE_ORB_H

#include "makeronline_base.h"
#include "orb/ORBmatcher.h"

namespace calibcamodo {

class MakerOnlineOrb : public MakerOnlineBase
{
public:
    MakerOnlineOrb(DatasetOrb* _pDatasetOrb);

    virtual bool DoMakeOnce();

    //!
    //! \brief Make and init new mappoints,
    //! which are introduced by kfnow.
    //!
    void RenewMpNow();

protected:
    DatasetOrb* mpDatasetOrb;
    ORBmatcher mOrbMatcher;
};

}
#endif // MAKERONLINE_ORB_H
