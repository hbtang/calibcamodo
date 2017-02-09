#include "edge_calibcamodo.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include <Eigen/LU>

using namespace Eigen;
using namespace std;

namespace g2o {

EdgeOptMk::EdgeOptMk() :
    BaseMultiEdge<3, Vector3D>() {
    resize(3);
}

void EdgeOptMk::computeError() {
    const VertexSE2* baseFrame          = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexPointXYZ* markPoint     = static_cast<const VertexPointXYZ*>(_vertices[1]);
    const VertexSE3* cameraOffset       = static_cast<const VertexSE3*>(_vertices[2]);

    SE2 se2_wb = baseFrame->estimate();
    Vector3D xyz_wm = markPoint->estimate();
    Isometry3D iso3_bc = cameraOffset->estimate();
    Vector3D xyz_cm_measure = _measurement;
    Isometry2D iso2_wb = se2_wb.toIsometry();

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
    xyz1_cm_measure << xyz_cm_measure(0), xyz_cm_measure(1), xyz_cm_measure(2), 1;
    Vector4D delta = (T3_wb*T3_bc).inverse() * xyz1_wm - xyz1_cm_measure;
    _error << delta(0), delta(1), delta(2);
}

}

namespace g2o {

EdgeVSlam::EdgeVSlam() :
    BaseBinaryEdge<2, Vector2D, VertexSE2, VertexPointXYZ>() {
    resize(2);
}

void EdgeVSlam::computeError() {
    const ParameterCamera* paramCam
            = static_cast<const ParameterCamera*>(parameter(0));
    const VertexSE2* baseFrame = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexPointXYZ* markPoint = static_cast<const VertexPointXYZ*>(_vertices[1]);

    Matrix3D K = paramCam->Kcam();
    Isometry3D Iso3_bc = paramCam->offset();

    SE2 se2_wb = baseFrame->estimate();
    Isometry2D iso2_wb = se2_wb.toIsometry();
    Matrix3D T2_wb = iso2_wb.matrix();
    Matrix4D T3_wb;
    T3_wb << T2_wb(0,0), T2_wb(0,1), 0, T2_wb(0,2),
            T2_wb(1,0), T2_wb(1,1), 0, T2_wb(1,2),
            0, 0, 1, 0,
            0, 0, 0, 1;

    Matrix4D T3_bc = Iso3_bc.matrix();
    Vector3D xyz_wm = markPoint->estimate();

    Matrix4D T3_cw = T3_bc.inverse() * T3_wb.inverse();
    Matrix3D R3_cw = T3_cw.block<3,3>(0,0);
    Vector3D xyz_cw = T3_cw.block<3,1>(0,3);

    Vector3D uvbar_cm = K * (R3_cw*xyz_wm + xyz_cw);
    Vector2D uv_cm;
    uv_cm << uvbar_cm(0)/uvbar_cm(2), uvbar_cm(1)/uvbar_cm(2);
    Vector2D uv_cm_measure = _measurement;

    _error = uv_cm - uv_cm_measure;
}

}


