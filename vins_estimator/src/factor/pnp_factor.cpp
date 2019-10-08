#include "pnp_factor.h"

Eigen::Matrix2d PnPFactor::sqrt_info;

PnPFactor::PnPFactor(const Eigen::Vector3d &_pt_3d, const Eigen::Vector3d &_pt_2d) : pt_3d(_pt_3d), pt_2d(_pt_2d)
{
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pt_2d.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

bool PnPFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d pt_imu_i = Qi.inverse() * (pt_3d - Pi);
    Eigen::Vector3d pt_camera_i = qic.inverse() * (pt_imu_i - tic);
    // std::cout << "pt_camera_i is " << pt_camera_i.transpose() << '\n';
    // std::cout << "pt_2d is " << pt_2d.transpose() << '\n';
    Eigen::Map<Eigen::Vector2d> residual(residuals);
#ifdef UNIT_SPHERE_ERROR 
    residual =  tangent_base * (pt_camera_i.normalized() - pt_2d.normalized());
#else
    double dep_i = pt_camera_i.z();
    residual = (pt_camera_i / dep_i).head<2>() - pt_2d.head<2>();
#endif
    residual = sqrt_info * residual;

    // std::cout << "residual is " << residual.transpose() << '\n';

    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
#ifdef UNIT_SPHERE_ERROR
        double norm = pt_camera_i.norm();
        Eigen::Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = pt_camera_i(0);
        x2 = pt_camera_i(1);
        x3 = pt_camera_i(2);
        norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), - x1 * x2 / pow(norm, 3),            - x1 * x3 / pow(norm, 3),
                     - x1 * x2 / pow(norm, 3),            1.0 / norm - x2 * x2 / pow(norm, 3), - x2 * x3 / pow(norm, 3),
                     - x1 * x3 / pow(norm, 3),            - x2 * x3 / pow(norm, 3),            1.0 / norm - x3 * x3 / pow(norm, 3);
        reduce = tangent_base * norm_jaco;
#else
        reduce << 1. / dep_i, 0, -pt_camera_i(0) / (dep_i * dep_i),
            0, 1. / dep_i, -pt_camera_i(1) / (dep_i * dep_i);
#endif
        reduce = sqrt_info * reduce;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * -Ri.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pt_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = -ric.transpose();
            jaco_ex.rightCols<3>() = Utility::skewSymmetric(pt_camera_i);
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
    }

    return true;
}