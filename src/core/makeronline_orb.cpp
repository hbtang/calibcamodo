#include "makeronline_orb.h"

namespace calibcamodo {

MakerOnlineOrb::MakerOnlineOrb(DatasetOrb* _pDatasetOrb):
    MakerOnlineBase(_pDatasetOrb), mpDatasetOrb(_pDatasetOrb) {

}

bool MakerOnlineOrb::DoMakeOnce() {
    return RenewKfNow();
}

}
