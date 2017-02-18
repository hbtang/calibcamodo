#include "edge_calibcamodo.h"
#include "g2o_math.h"

#include "core/adapter.h"

#include <Eigen/LU>

using namespace Eigen;
using namespace std;
using namespace calibcamodo;

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
    paramCam = 0;
    resizeParameters(1);
    installParameter(paramCam, 0);
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

void EdgeVSlam::linearizeOplus() {

    BaseBinaryEdge<2, Vector2D, VertexSE2, VertexPointXYZ>::linearizeOplus();
//    cerr << "base::_jacobianOplusXi" << endl << _jacobianOplusXi << endl;
//    cerr << "base::_jacobianOplusXj" << endl << _jacobianOplusXj << endl;

//    const ParameterCamera* paramCam
//            = static_cast<const ParameterCamera*>(parameter(0));
//    const VertexSE2* baseFrame = static_cast<const VertexSE2*>(_vertices[0]);
//    const VertexPointXYZ* markPoint = static_cast<const VertexPointXYZ*>(_vertices[1]);

//    SE3Quat se3bc = toG2oSE3Quat(paramCam->offset());
//    SE3Quat se3cb = se3bc.inverse();
//    Matrix4D Tcb = toTransMat(se3cb);

//    Vector3D pwm = markPoint->estimate();
//    SE2 se2wb = baseFrame->estimate();
//    SE3Quat se3wb = toG2oSE3Quat(se2wb);
//    SE3Quat se3bw = se3wb.inverse();

//    Matrix<double,4,6,Eigen::ColMajor> J_pcm_xiwb_bar = - Tcb * dcircle(se3bw*pwm) * JJl(se3bw);
//    Matrix<double,3,6,Eigen::ColMajor> J_pcm_xiwb = J_pcm_xiwb_bar.topRows(3);
//    Matrix<double,3,3,Eigen::ColMajor> J_pcm_v3wb = J_pcm_xiwb * JacobianSE3SE2(se3wb);


////    cerr << "Tcb" << endl << Tcb << endl;
////    cerr << "dcircle(se3bw*pwm)" << endl << dcircle(se3bw*pwm) << endl;
////    cerr << "JJl(se3bw)" << endl << JJl(se3bw) << endl;
////    cerr << "J_pcm_xiwb" << endl << J_pcm_xiwb << endl;
////    cerr << "J_pcm_v3wb" << endl << J_pcm_v3wb << endl;
////    cerr << "JacobianSE3SE2(se3wb)" << endl << JacobianSE3SE2(se3wb) << endl;

//    Vector3D pcm = se3cb*se3bw*pwm;
//    Matrix3D K = paramCam->Kcam();
//    Matrix<double,2,3,Eigen::ColMajor> J_uv_pcm = JacobianUV2XYZ(pcm, K);

//    SE3Quat se3cw = se3cb*se3bw;
//    Matrix3D Rcw = toG2oIsometry3D(se3cw).rotation();

//    _jacobianOplusXi = J_uv_pcm*J_pcm_v3wb;
//    _jacobianOplusXj = J_uv_pcm*Rcw;

////    double tmp = _jacobianOplusXi(0,0);
////    if(std::isnan(tmp)) {
////        cerr << "nan!" << endl;
////    }
////    cerr << "derived::_jacobianOplusXi" << endl << _jacobianOplusXi << endl;
////    cerr << "derived::_jacobianOplusXj" << endl << _jacobianOplusXj << endl;

    return;
}

}

namespace g2o {

EdgeVSclam::EdgeVSclam() :
    BaseMultiEdge<2, Vector2D>() {
    resize(3);
    _camParam = 0;
    resizeParameters(1);
    installParameter(_camParam, 0);
}

void EdgeVSclam::computeError() {
    const CameraParameters* camParam
            = static_cast<const CameraParameters*>(parameter(0));
    const VertexSE2* baseFrame = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexPointXYZ* markPoint = static_cast<const VertexPointXYZ*>(_vertices[1]);
    const VertexSE3* cameraOffset = static_cast<const VertexSE3*>(_vertices[2]);

    SE3Quat se3wb = toG2oSE3Quat(baseFrame->estimate());
    SE3Quat se3bc = toG2oSE3Quat(cameraOffset->estimate());
    Vector3D pwm = markPoint->estimate();
    SE3Quat se3cw = (se3wb*se3bc).inverse();
    _error = camParam->cam_map(se3cw.map(pwm)) - _measurement;

    return;
}

}


