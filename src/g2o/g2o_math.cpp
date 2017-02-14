#include "g2o_math.h"

namespace g2o {

using namespace std;
using namespace Eigen;

Matrix3D Jl(const Vector3D& v) {
    double theta = v.norm();
    if(theta == 0) {
        return Matrix3D::Identity();
    }
    double invtheta = 1. / theta;
    double sint = std::sin(theta);
    double cost = std::cos(theta);
    Vector3D a = v * invtheta;
    Matrix3D Jl =
            sint * invtheta * Matrix3D::Identity()
            + ( 1 - sint * invtheta) * a * a.transpose()
            + ( 1 - cost ) * invtheta * skew(a);
    return Jl;
}

Matrix3D JlInv(const Vector3D &v) {
    double theta = v.norm();
    if(theta == 0) {
        return Matrix3D::Identity();
    }
    double thetahalf = theta * 0.5;
    double invtheta = 1. / theta;
    double cothalf = std::tan(M_PI_2 - thetahalf);
    Vector3D a = v * invtheta;
    Matrix3D invJl =
            thetahalf * cothalf * Matrix3D::Identity()
            + (1 - thetahalf * cothalf) * a * a.transpose()
            - thetahalf * skew(a);
    return invJl;
}

Eigen::Matrix<double,4,6,Eigen::ColMajor> dcircle(const Vector4D& v) {
    Eigen::Matrix<double,4,6,Eigen::ColMajor> m;
    m.fill(0.0);
    double ita = v(3);
    Vector3D eps; eps << v(0),v(1),v(2);
    m.block<3,3>(0,0) = ita * Matrix3D::Identity();
    m.block<3,3>(0,3) = -skew(eps);
    return m;
}

Eigen::Matrix<double,4,6,Eigen::ColMajor> dcircle(const Vector3D& v) {
    Vector4D vbar;
    vbar << v(0),v(1),v(2),1;
    return dcircle(vbar);
}

Eigen::Matrix<double,6,3,Eigen::ColMajor> JacobianSE3SE2(const Vector6D& vlie) {
    // v: 6d vector of lie algebra
    // v[0:2], rho translation; v[3:5] phi rotation

    //    Vector3D rho = v.block<3,1>(0,0);
    Vector3D phi = vlie.block<3,1>(3,0);
    Matrix3D J_rho_r = JlInv(phi);
    Matrix<double,3,2,Eigen::ColMajor> J_rho_xy = J_rho_r.block<3,2>(0,0);
    Eigen::Matrix<double,6,3,Eigen::ColMajor> matRet;
    matRet.fill(0.0);
    matRet.block<3,2>(0,0) = J_rho_xy;
    matRet(5,2) = 1;
    return matRet;
}

Eigen::Matrix<double,6,3,Eigen::ColMajor> JacobianSE3SE2(const SE3Quat& se3) {
    Vector6D xi = se3.log();    // in g2o format: xi[0:2] is omega/phi, xi[3:5] is upsilon/rho
    Vector3D phi = xi.topRows(3);
    //    Vector3D rho = xi.bottomRows(3);
    Matrix3D J_rho_r = JlInv(phi);
    Matrix<double,3,2,Eigen::ColMajor> J_rho_xy = J_rho_r.leftCols(2);

    Vector3D v = se3.translation();
    double x = v(0);
    double y = v(1);
    double theta = phi(2);
    Vector3D J_rho_theta = JacobianRhoTheta(x, y, theta);

    Eigen::Matrix<double,6,3,Eigen::ColMajor> J_xi_xyt;
    J_xi_xyt.fill(0.0);
    J_xi_xyt.topLeftCorner(3,2) = J_rho_xy;
    J_xi_xyt.topRightCorner(3,1) = J_rho_theta;
    J_xi_xyt(5,2) = 1;
    return J_xi_xyt;
}

Eigen::Matrix<double,2,3,Eigen::ColMajor> JacobianUV2XYZ(const Vector3D& pcm, const Matrix3D& K) {
    Matrix3D tmp;
    double x = pcm(0);
    double y = pcm(1);
    double z = pcm(2);
    tmp <<  1/z, 0, -x/(z*z),
            0, 1/z, -y/(z*z),
            0, 0, 0;
    //    Matrix3D J = K*tmp;
    return (K*tmp).topRows(2);
}

Matrix6D JJl(const SE3Quat& se3) {
    Vector6D tmp = se3.log();
    Vector6D v;
    v.topRows(3) = tmp.bottomRows(3);
    v.bottomRows(3) = tmp.topRows(3);
    return JJl(v);
}

Matrix6D JJl(const Vector6D& v) {
    return JJlInv(v).inverse();
}

Matrix6D JJlInv(const Vector6D& v) {

    //! rho: translation; phi: rotation
    //! vector order: [trans, rot]

    Vector3D rho, phi;
    for(int i = 0; i < 3; i++) {
        phi[i] = v[i+3];
        rho[i] = v[i];
    }
    double theta = phi.norm();
    if(theta == 0) {
        return Matrix6D::Identity();
    }

    Matrix3D Phi = skew(phi);
    Matrix3D Rho = skew(rho);
    double sint = sin(theta);
    double cost = cos(theta);
    double theta2 = theta * theta;
    double theta3 = theta * theta2;
    double theta4 = theta2 * theta2;
    double theta5 = theta4  * theta;
    double invtheta = 1./theta;
    double invtheta3 = 1./theta3;
    double invtheta4 = 1./theta4;
    double invtheta5 = 1./theta5;
    Matrix3D PhiRho = Phi * Rho;
    Matrix3D RhoPhi = Rho * Phi;
    Matrix3D PhiRhoPhi = PhiRho * Phi;
    Matrix3D PhiPhiRho = Phi * PhiRho;
    Matrix3D RhoPhiPhi = RhoPhi * Phi;
    Matrix3D PhiRhoPhiPhi = PhiRhoPhi * Phi;
    Matrix3D PhiPhiRhoPhi = Phi * PhiRhoPhi;

    double temp = (1. - 0.5 * theta2 - cost) * invtheta4;

    Matrix3D Ql =
            0.5 * Rho + (theta - sint) * invtheta3 * (PhiRho + RhoPhi + PhiRhoPhi)
            - temp * (PhiPhiRho + RhoPhiPhi -3. * PhiRhoPhi)
            - 0.5 * (temp - ( 3. * (theta - sint) + theta3 * 0.5) * invtheta5 ) * (PhiRhoPhiPhi + PhiPhiRhoPhi);

    double thetahalf = theta * 0.5;
    double cothalf = tan(M_PI_2 - thetahalf);
    Vector3D a = phi * invtheta;
    Matrix3D invJl =
            thetahalf * cothalf * Matrix3D::Identity()
            + (1 - thetahalf * cothalf) * a * a.transpose()
            - thetahalf * skew(a);

    Matrix6D invJJl = Matrix6D::Zero();
    invJJl.block<3,3>(0,0) = invJl;
    invJJl.block<3,3>(0,3) = - invJl * Ql * invJl;
    invJJl.block<3,3>(3,3) = invJl;
    return invJJl;
}

Vector3D JacobianRhoTheta(double x, double y, double theta) {
    if(theta == 0) {
        return Vector3D::Zero();
    }

    double tmp = (theta-sin(theta))/(2*(cos(theta)-1));
    Matrix3D I = Matrix3D::Identity();
    Matrix3D ahat;
    ahat << 0, -1, 0,
            1, 0, 0,
            0, 0, 0;
    Matrix3D aat;
    aat <<  0, 0, 0,
            0, 0, 0,
            0, 0, 1;
    Matrix3D dJlInv_dtheta = tmp*I + (1-tmp)*aat - 0.5*ahat;

    Vector3D v;
    v << x, y, theta;
    return dJlInv_dtheta*v;
}

}
