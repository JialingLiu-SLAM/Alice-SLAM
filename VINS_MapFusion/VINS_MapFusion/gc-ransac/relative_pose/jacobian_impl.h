#ifndef POSELIB_JACOBIAN_IMPL_H_
#define POSELIB_JACOBIAN_IMPL_H_

//#include "essential.h"

namespace pose_lib {

template <typename CameraModel, typename LossFunction>
class CameraJacobianAccumulator 
{
    public:
        CameraJacobianAccumulator(
            const cv::Mat& correspondences_,
            const size_t* sample_,
            const size_t& sample_size_,
            const Camera &cam,
            const LossFunction &loss,
            const double *w = nullptr) : 
                correspondences(&correspondences_), 
                sample(sample_),
                sample_size(sample_size_),
                loss_fn(loss),
                camera(cam),
                weights(w) {}

        double residual(const CameraPose &pose) const 
        {
            double cost = 0;
            Eigen::Vector2d x;
            Eigen::Vector3d X;
            size_t rowIdx = 0;
            for (size_t i = 0; i < correspondences->rows; ++i) 
            {
                if (sample == nullptr)
                    rowIdx = i;
                else
                    rowIdx = sample[i];
                
                x(0) = correspondences->at<double>(rowIdx, 0);
                x(1) = correspondences->at<double>(rowIdx, 1);
                X(0) = correspondences->at<double>(rowIdx, 2);
                X(1) = correspondences->at<double>(rowIdx, 3);
                X(2) = correspondences->at<double>(rowIdx, 4);

                const Eigen::Vector3d Z = pose.apply(X);
                // Note this assumes points that are behind the camera will stay behind the camera
                // during the optimization
                if (Z(2) < 0)
                    continue;
                const double inv_z = 1.0 / Z(2);
                Eigen::Vector2d p(Z(0) * inv_z, Z(1) * inv_z);
                CameraModel::project(camera.params, p, &p);
                const double r0 = p(0) - x(0);
                const double r1 = p(1) - x(1);
                const double r_squared = r0 * r0 + r1 * r1;
                cost += weights[i] * loss_fn.loss(r_squared);
            }
            return cost;
        }

        // computes J.transpose() * J and J.transpose() * res
        // Only computes the lower half of JtJ
        size_t accumulate(
            const CameraPose &pose, 
            Eigen::Matrix<double, 6, 6> &JtJ,
            Eigen::Matrix<double, 6, 1> &Jtr) const 
        {
            Eigen::Matrix3d R = pose.R();
            Eigen::Matrix2d Jcam;
            Jcam.setIdentity(); // we initialize to identity here (this is for the calibrated case)
            size_t num_residuals = 0;

            Eigen::Vector2d x;
            Eigen::Vector3d X;
            size_t rowIdx = 0;
            for (size_t i = 0; i < correspondences->rows; ++i) 
            {
                if (sample == nullptr)
                    rowIdx = i;
                else
                    rowIdx = sample[i];

                x(0) = correspondences->at<double>(rowIdx, 0);
                x(1) = correspondences->at<double>(rowIdx, 1);
                X(0) = correspondences->at<double>(rowIdx, 2);
                X(1) = correspondences->at<double>(rowIdx, 3);
                X(2) = correspondences->at<double>(rowIdx, 4);
            
                const Eigen::Vector3d Z = R * X + pose.t;
                const Eigen::Vector2d z = Z.hnormalized();

                // Note this assumes points that are behind the camera will stay behind the camera
                // during the optimization
                if (Z(2) < 0)
                    continue;

                // Project with intrinsics
                Eigen::Vector2d zp = z;
                CameraModel::project_with_jac(camera.params, z, &zp, &Jcam);

                // Setup residual
                Eigen::Vector2d r = zp - x;
                const double r_squared = r.squaredNorm();
                const double weight = weights[i] * loss_fn.weight(r_squared);

                if (weight == 0.0) {
                    continue;
                }
                num_residuals++;

                // Compute jacobian w.r.t. Z (times R)
                Eigen::Matrix<double, 2, 3> dZ;
                dZ.block<2, 2>(0, 0) = Jcam;
                dZ.col(2) = -Jcam * z;
                dZ *= 1.0 / Z(2);
                dZ *= R;

                const double X0 = X(0);
                const double X1 = X(1);
                const double X2 = X(2);
                const double dZtdZ_0_0 = weight * dZ.col(0).dot(dZ.col(0));
                const double dZtdZ_1_0 = weight * dZ.col(1).dot(dZ.col(0));
                const double dZtdZ_1_1 = weight * dZ.col(1).dot(dZ.col(1));
                const double dZtdZ_2_0 = weight * dZ.col(2).dot(dZ.col(0));
                const double dZtdZ_2_1 = weight * dZ.col(2).dot(dZ.col(1));
                const double dZtdZ_2_2 = weight * dZ.col(2).dot(dZ.col(2));
                JtJ(0, 0) += X2 * (X2 * dZtdZ_1_1 - X1 * dZtdZ_2_1) + X1 * (X1 * dZtdZ_2_2 - X2 * dZtdZ_2_1);
                JtJ(1, 0) += -X2 * (X2 * dZtdZ_1_0 - X0 * dZtdZ_2_1) - X1 * (X0 * dZtdZ_2_2 - X2 * dZtdZ_2_0);
                JtJ(2, 0) += X1 * (X0 * dZtdZ_2_1 - X1 * dZtdZ_2_0) - X2 * (X0 * dZtdZ_1_1 - X1 * dZtdZ_1_0);
                JtJ(3, 0) += X1 * dZtdZ_2_0 - X2 * dZtdZ_1_0;
                JtJ(4, 0) += X1 * dZtdZ_2_1 - X2 * dZtdZ_1_1;
                JtJ(5, 0) += X1 * dZtdZ_2_2 - X2 * dZtdZ_2_1;
                JtJ(1, 1) += X2 * (X2 * dZtdZ_0_0 - X0 * dZtdZ_2_0) + X0 * (X0 * dZtdZ_2_2 - X2 * dZtdZ_2_0);
                JtJ(2, 1) += -X2 * (X1 * dZtdZ_0_0 - X0 * dZtdZ_1_0) - X0 * (X0 * dZtdZ_2_1 - X1 * dZtdZ_2_0);
                JtJ(3, 1) += X2 * dZtdZ_0_0 - X0 * dZtdZ_2_0;
                JtJ(4, 1) += X2 * dZtdZ_1_0 - X0 * dZtdZ_2_1;
                JtJ(5, 1) += X2 * dZtdZ_2_0 - X0 * dZtdZ_2_2;
                JtJ(2, 2) += X1 * (X1 * dZtdZ_0_0 - X0 * dZtdZ_1_0) + X0 * (X0 * dZtdZ_1_1 - X1 * dZtdZ_1_0);
                JtJ(3, 2) += X0 * dZtdZ_1_0 - X1 * dZtdZ_0_0;
                JtJ(4, 2) += X0 * dZtdZ_1_1 - X1 * dZtdZ_1_0;
                JtJ(5, 2) += X0 * dZtdZ_2_1 - X1 * dZtdZ_2_0;
                JtJ(3, 3) += dZtdZ_0_0;
                JtJ(4, 3) += dZtdZ_1_0;
                JtJ(5, 3) += dZtdZ_2_0;
                JtJ(4, 4) += dZtdZ_1_1;
                JtJ(5, 4) += dZtdZ_2_1;
                JtJ(5, 5) += dZtdZ_2_2;
                r *= weight;
                Jtr(0) += (r(0) * (X1 * dZ(0, 2) - X2 * dZ(0, 1)) + r(1) * (X1 * dZ(1, 2) - X2 * dZ(1, 1)));
                Jtr(1) += (-r(0) * (X0 * dZ(0, 2) - X2 * dZ(0, 0)) - r(1) * (X0 * dZ(1, 2) - X2 * dZ(1, 0)));
                Jtr(2) += (r(0) * (X0 * dZ(0, 1) - X1 * dZ(0, 0)) + r(1) * (X0 * dZ(1, 1) - X1 * dZ(1, 0)));
                Jtr(3) += (dZ(0, 0) * r(0) + dZ(1, 0) * r(1));
                Jtr(4) += (dZ(0, 1) * r(0) + dZ(1, 1) * r(1));
                Jtr(5) += (dZ(0, 2) * r(0) + dZ(1, 2) * r(1));
            }
            return num_residuals;
        }

        CameraPose step(Eigen::Matrix<double, 6, 1> dp, const CameraPose &pose) const 
        {
            CameraPose pose_new;
            // The rotation is parameterized via the lie-rep. and post-multiplication
            //   i.e. R(delta) = R * expm([delta]_x)
            pose_new.q = pose.quat_step_post(pose.q, dp.block<3, 1>(0, 0));

            // Translation is parameterized as (negative) shift in position
            //  i.e. t(delta) = t + R*delta
            pose_new.t = pose.t + pose.rotate(dp.block<3, 1>(3, 0));
            return pose_new;
        }
        
        typedef CameraPose param_t;
        static constexpr size_t num_params = 6;

    private:
        const cv::Mat* correspondences;
        const size_t* sample;
        const size_t sample_size;

        const Camera &camera;
        const LossFunction &loss_fn;
        const double *weights;
};



} // namespace pose_lib

#endif
