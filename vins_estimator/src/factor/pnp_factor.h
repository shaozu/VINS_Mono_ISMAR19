#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class PnPFactor : public ceres::SizedCostFunction<2, 7, 7>
{
public:
    PnPFactor(const Eigen::Vector3d &_pt_3d, const Eigen::Vector3d &_pt_2d);
    Eigen::Matrix<double, 3, 4> QuaternionDerivation(const Eigen::Quaterniond &q0, const Eigen::Vector3d &point) const;

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d pt_3d, pt_2d;
    static Eigen::Matrix2d sqrt_info;
    Eigen::Matrix<double, 2, 3> tangent_base;
};

struct PnPFactor2
{
    PnPFactor2(const Eigen::Vector3d &_pt_3d, const Eigen::Vector3d &_pt_2d)
        : pt_3d(_pt_3d), pt_2d(_pt_2d) {}

    template <typename T>
    bool operator()(const T *pose, const T *ex_pose, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> w_pt_3d{T(pt_3d.x()), T(pt_3d.y()), T(pt_3d.z())};
        Eigen::Matrix<T, 3, 1> obs_pt_2d{T(pt_2d.x()), T(pt_2d.y()), T(pt_2d.z())};
        Eigen::Matrix<T, 3, 1> t_w_i{pose[0], pose[1], pose[2]};
        Eigen::Quaternion<T> q_w_i{pose[6], pose[3], pose[4], pose[5]};
        Eigen::Matrix<T, 3, 1> t_i_c{ex_pose[0], ex_pose[1], ex_pose[2]};
        Eigen::Quaternion<T> q_i_c{ex_pose[6], ex_pose[3], ex_pose[4], ex_pose[5]};

        Eigen::Matrix<T, 3, 1> pt_i = q_w_i.inverse() * (w_pt_3d - t_w_i);
        Eigen::Matrix<T, 3, 1> pt_c = q_i_c.inverse() * (pt_i - t_i_c);
        Eigen::Matrix<T, 2, 1> res{pt_c.x()/pt_c.z() - obs_pt_2d.x(), pt_c.y()/pt_c.z() - obs_pt_2d.y()};
        Eigen::Matrix<T, 2, 2> t_sqrt_info;
        t_sqrt_info(0, 0) = T(sqrt_info(0, 0)); t_sqrt_info(0, 1) = T(sqrt_info(0, 1));
        t_sqrt_info(1, 0) = T(sqrt_info(1, 0)); t_sqrt_info(1, 1) = T(sqrt_info(1, 1));
        res = t_sqrt_info * res;
        residual[0] = res[0];
        residual[1] = res[1];
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d _pt_3d, const Eigen::Vector3d _pt_2d)
    {
        return (new ceres::AutoDiffCostFunction<PnPFactor2, 2, 7, 7>(new PnPFactor2(_pt_3d, _pt_2d)));
    }

    Eigen::Vector3d pt_3d, pt_2d;
    static Eigen::Matrix2d sqrt_info;
};
