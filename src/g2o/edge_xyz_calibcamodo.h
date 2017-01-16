#ifndef G2O_EDGE_XYZ_CALIBCAMODO
#define G2O_EDGE_XYZ_CALIBCAMODO

#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

namespace g2o {

class EdgeXYZCalibCamOdo : public BaseMultiEdge<3, Vector3D>
{
public:
    EdgeXYZCalibCamOdo();
    void computeError();
    void setMeasurement(const Vector3D& m){
        _measurement = m;
    }
    virtual bool read(std::istream& is) {return false;}
    virtual bool write(std::ostream& os) const {return false;}
protected:
};

} // end namespace

#endif
