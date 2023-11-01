// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

#pragma once

// system header
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t);

Eigen::Matrix3d skew(Eigen::Vector3d& mat_in);

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
  public:
    EdgeAnalyticCostFunction(Eigen::Vector3d curr_point, Eigen::Vector3d last_point_a, Eigen::Vector3d last_point_b);
    virtual ~EdgeAnalyticCostFunction() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d curr_point_;
    Eigen::Vector3d last_point_a_;
    Eigen::Vector3d last_point_b_;
};

class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
  public:
    SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point, Eigen::Vector3d plane_unit_norm, double negative_OA_dot_norm);
    virtual ~SurfNormAnalyticCostFunction() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d curr_point_;
    Eigen::Vector3d plane_unit_norm_;
    double negative_OA_dot_norm_;
};

class PoseSE3Parameterization : public ceres::Manifold
{
  public:
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool PlusJacobian(const double*, double* jacobian) const;
    virtual bool RightMultiplyByPlusJacobian(const double*, const int,
                                             const double*, double*) const;
    virtual bool Minus(const double* y, const double* x, double* y_minus_x) const;
    virtual bool MinusJacobian(const double*, double* jacobian) const;
    virtual int AmbientSize() const { return 7; }
    virtual int TangentSize() const { return 6; }
};
