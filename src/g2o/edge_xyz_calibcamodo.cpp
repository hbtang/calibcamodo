#include "edge_xyz_calibcamodo.h"

namespace g2o {

EdgeXYZCalibCamOdo::EdgeXYZCalibCamOdo() :
    BaseMultiEdge<3, Vector3D>() {
    resize(3);
}

void EdgeXYZCalibCamOdo::computeError() {
    const VertexSE2* baseFrame          = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexPointXYZ* markPoint     = static_cast<const VertexPointXYZ*>(_vertices[1]);
    const VertexSE3* cameraOffset       = static_cast<const VertexSE3*>(_vertices[2]);

    const SE2& se2_wb = baseFrame->estimate();
    const Vector3D& xyz_wm = markPoint->estimate();
    const Isometry3D& iso3_bc = cameraOffset->estimate();
    const Vector3D& xyz_cm_measure = _measurement;
    const Isometry2D iso2_wb = se2_wb.toIsometry();

    Matrix4D T3_bc = iso3_bc.matrix();
    Matrix3D T2_wb = iso2_wb.matrix();
    Matrix4D T3_wb;
    T3_wb << T2_wb(0,0), T2_wb(0,1), 0, T2_wb(0,2),
            T2_wb(1,0), T2_wb(1,1), 0, T2_wb(1,2),
            0, 0, 1, 0,
            0, 0, 0, 1;

    Vector4D xyz1_wm;
    xyz1_wm << xyz_wm(0), xyz_wm(1), xyz_wm(2), 1;
    Vector4D xyz1_cm_measure;
    xyz1_wm << xyz_cm_measure(0), xyz_cm_measure(1), xyz_cm_measure(2), 1;
    Vector4D delta = (T3_wb*T3_bc).inverse() * xyz1_wm - xyz1_cm_measure;
    _error << delta(0), delta(1), delta(2);
}


} // end namespace
