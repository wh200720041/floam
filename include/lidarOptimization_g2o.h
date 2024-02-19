#pragma once
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../3rdparty/sophus/se3.hpp"

namespace TESTG2O
{
    class FLOAMVertex : public g2o::BaseVertex<6, Sophus::SE3d> //  顶点维度  顶点类型
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void setToOriginImpl() override
        {
            _estimate = Sophus::SE3d();
        }
        virtual void oplusImpl(const double *update) override
        {
            Eigen::Matrix<double, 6, 1> delta_r;
            delta_r << update[0], update[1], update[2], update[3], update[4], update[5];

            _estimate = Sophus::SE3d::exp(delta_r) * _estimate; //  左乘
            // _estimate = _estimate * Sophus::SE3d::exp(delta_r); //  右乘
        }
        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}
    };

    class FLOAMEdge : public g2o::BaseUnaryEdge<1, double, FLOAMVertex> //  观测值维度   观测值类型   边类型
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        FLOAMEdge(Eigen::Vector3d pa, Eigen::Vector3d pb, Eigen::Vector3d cp) : BaseUnaryEdge(), lp_a(pa), lp_b(pb), c_p(cp) {}

        virtual void computeError() override
        {
            const FLOAMVertex *v = static_cast<const FLOAMVertex *>(_vertices[0]);
            const Sophus::SE3d T = v->estimate();
            Eigen::Vector3d lp = T * c_p;
            Eigen::Vector3d nu = (lp - lp_a).cross(lp - lp_b);
            Eigen::Vector3d de = lp_a - lp_b;
            double de_norm = de.norm();
            double nu_norm = nu.norm();

            _error(0, 0) = nu_norm / de_norm;
        }

        virtual void linearizeOplus() override
        {
            const FLOAMVertex *v = static_cast<const FLOAMVertex *>(_vertices[0]);
            const Sophus::SE3d T = v->estimate();
            //  de/T的李代数
            Eigen::Matrix3d skew_lp = Sophus::SO3d::hat(T * c_p); //  左乘扰动
            Eigen::Vector3d lp = T * c_p;
            Eigen::Vector3d nu = (lp - lp_a).cross(lp - lp_b);
            Eigen::Vector3d de = lp_a - lp_b;
            double de_norm = de.norm();
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            (dp_by_se3.block<3, 3>(0, 0)).setIdentity();
            dp_by_se3.block<3, 3>(0, 3) = -skew_lp;
            Eigen::Matrix3d skew_de = Sophus::SO3d::hat(lp_a - lp_b);
            _jacobianOplusXi.block<1, 6>(0, 0) = -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
        }

        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}

    private:
        Eigen::Vector3d lp_a, lp_b, c_p;
    };

    class FLOAMSurf : public g2o::BaseUnaryEdge<1, double, FLOAMVertex> //  观测值维度   观测值类型   边类型
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        FLOAMSurf(Eigen::Vector3d cur_p, Eigen::Vector3d p_nor) : BaseUnaryEdge(), c_p(cur_p), p_n(p_nor) {}

        virtual void computeError() override
        {
            const FLOAMVertex *v = static_cast<const FLOAMVertex *>(_vertices[0]);
            const Sophus::SE3d T = v->estimate();
            Eigen::Vector3d point_w = T * c_p;

            _error(0, 0) = p_n.dot(point_w) + _measurement;
        }

        virtual void linearizeOplus() override
        {
            const FLOAMVertex *v = static_cast<const FLOAMVertex *>(_vertices[0]);
            const Sophus::SE3d T = v->estimate();
            //  de/T的李代数
            Eigen::Matrix3d skew_point_w = Sophus::SO3d::hat(T * c_p); //  左乘扰动
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            (dp_by_se3.block<3, 3>(0, 0)).setIdentity();
            dp_by_se3.block<3, 3>(0, 3) = -skew_point_w;
            _jacobianOplusXi.block<1, 6>(0, 0) = p_n.transpose() * dp_by_se3;
        }

        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}

    private:
        Eigen::Vector3d c_p, p_n;
    };
}