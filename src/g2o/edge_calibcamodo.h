#ifndef G2O_EDGE_CALIBCAMODO
#define G2O_EDGE_CALIBCAMODO

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o/types/sba/types_six_dof_expmap.h"


namespace g2o {

class EdgeOptMk : public BaseMultiEdge<3, Vector3D> {
public:
    EdgeOptMk();
    void computeError();
    void setMeasurement(const Vector3D& m) {
        _measurement = m;
    }

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
};

class EdgeVSlam : public BaseBinaryEdge<2, Vector2D, VertexSE2, VertexPointXYZ> {
public:
    EdgeVSlam();
    void computeError();
    void setMeasurement(const Vector2D& m) {
        _measurement = m;
    }
    virtual void linearizeOplus();

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

private:
    const ParameterCamera* paramCam;

};

class EdgeVSclam : public BaseMultiEdge<2, Vector2D> {
public:
    EdgeVSclam();
    void computeError();
    void setMeasurement(const Vector2D& m) {
        _measurement = m;
    }

//    virtual void linearizeOplus() {
//        BaseMultiEdge<2, Vector2D>::linearizeOplus();
//        std::cerr << "_jacobianOplus[0]" << std::endl << _jacobianOplus[0] << std::endl;
//        std::cerr << "_jacobianOplus[1]" << std::endl << _jacobianOplus[1] << std::endl;
//        std::cerr << "_jacobianOplus[2]" << std::endl << _jacobianOplus[2] << std::endl;
//    }

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

private:
    const CameraParameters* _camParam;
};

} // end namespace

#endif
