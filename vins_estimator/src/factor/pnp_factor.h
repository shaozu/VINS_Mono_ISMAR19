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
    PnPFactor(const Eigen::Vector3d &_pt_3d);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d pt_3d, pt_2d;
};