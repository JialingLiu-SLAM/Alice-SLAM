#include "bundle.h"
#include "jacobian_impl.h"
#include "robust_loss.h"
#include "colmap_models.h"
#include <opencv2/core.hpp>

namespace pose_lib {

#define SWITCH_LOSS_FUNCTIONS                     \
    case BundleOptions::LossType::TRIVIAL:        \
        SWITCH_LOSS_FUNCTION_CASE(TrivialLoss);   \
        break;                                    \
    case BundleOptions::LossType::TRUNCATED:      \
        SWITCH_LOSS_FUNCTION_CASE(TruncatedLoss); \
        break;                                    \
    case BundleOptions::LossType::HUBER:          \
        SWITCH_LOSS_FUNCTION_CASE(HuberLoss);     \
        break;                                    \
    case BundleOptions::LossType::CAUCHY:         \
        SWITCH_LOSS_FUNCTION_CASE(CauchyLoss);    \
        break;

template <typename JacobianAccumulator>
int lm_pnp_impl(const JacobianAccumulator &accum, 
    CameraPose *pose, 
    const BundleOptions &opt) 
{
    Eigen::Matrix<double, 6, 6> JtJ;
    Eigen::Matrix<double, 6, 1> Jtr;
    double lambda = opt.initial_lambda;
    Eigen::Matrix3d sw;
    sw.setZero();

    // Compute initial cost
    double cost = accum.residual(*pose);
    bool recompute_jac = true;
    int iter;
    for (iter = 0; iter < opt.max_iterations; ++iter) {
        // We only recompute jacobian and residual vector if last step was successful
        if (recompute_jac) {
            JtJ.setZero();
            Jtr.setZero();
            accum.accumulate(*pose, JtJ, Jtr);
            if (Jtr.norm() < opt.gradient_tol) {
                break;
            }
        }

        // Add dampening
        JtJ(0, 0) += lambda;
        JtJ(1, 1) += lambda;
        JtJ(2, 2) += lambda;
        JtJ(3, 3) += lambda;
        JtJ(4, 4) += lambda;
        JtJ(5, 5) += lambda;

        Eigen::Matrix<double, 6, 1> sol = -JtJ.selfadjointView<Eigen::Lower>().llt().solve(Jtr);

        if (sol.norm() < opt.step_tol) {
            break;
        }

        Eigen::Vector3d w = sol.block<3, 1>(0, 0);
        const double theta = w.norm();
        w /= theta;
        const double a = std::sin(theta);
        const double b = std::cos(theta);
        sw(0, 1) = -w(2);
        sw(0, 2) = w(1);
        sw(1, 2) = -w(0);
        sw(1, 0) = w(2);
        sw(2, 0) = -w(1);
        sw(2, 1) = w(0);

        CameraPose pose_new;
        Eigen::Matrix3d R = pose->R();
        pose_new.q = pose_new.rotmat_to_quat(R + R * (a * sw + (1 - b) * sw * sw));
        pose_new.t = pose->t + R * sol.block<3, 1>(3, 0);
        double cost_new = accum.residual(pose_new);

        if (cost_new < cost) {
            *pose = pose_new;
            lambda /= 10;
            cost = cost_new;
            recompute_jac = true;
        } else {
            JtJ(0, 0) -= lambda;
            JtJ(1, 1) -= lambda;
            JtJ(2, 2) -= lambda;
            JtJ(3, 3) -= lambda;
            JtJ(4, 4) -= lambda;
            JtJ(5, 5) -= lambda;
            lambda *= 10;
            recompute_jac = false;
        }
    }

    return iter;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////
// Absolute pose with points (PnP)
// Interface for calibrated camera
int refine_pnp(
    const cv::Mat &correspondences_,
    const size_t *sample_,
    const size_t &sample_size_, 
    CameraPose *pose,
    const BundleOptions &opt,
    const double *weights) 
{
    pose_lib::Camera camera;
    camera.model_id = NullCameraModel::model_id;
    return refine_pnp(
            correspondences_, 
            sample_,
            sample_size_,
            camera, 
            pose, 
            opt, 
            weights);
}

template <typename CameraModel, typename LossFunction>
int refine_pnp(
    const cv::Mat &correspondences_,
    const size_t *sample_,
    const size_t &sample_size_, 
    const Camera &camera,
    CameraPose *pose,
    const BundleOptions &opt,
    const double *weights)
{                                      
    LossFunction loss_fn(opt.loss_scale);                                                              
    CameraJacobianAccumulator<CameraModel, LossFunction> accum(
        correspondences_, 
        sample_, 
        sample_size_, 
        loss_fn, 
        weights);

    return lm_pnp_impl<decltype(accum)>(accum, pose, opt);          
}

template <typename CameraModel>
int refine_pnp(
    const cv::Mat &correspondences_,
    const size_t *sample_,
    const size_t &sample_size_, 
    const Camera &camera,
    CameraPose *pose,
    const BundleOptions &opt,
    const double *weights) 
{
    switch (opt.loss_type) {
#define SWITCH_LOSS_FUNCTION_CASE(LossFunction)                                                                        \
    return refine_pnp<CameraModel, LossFunction>(                                                                      \
            correspondences_,                                                                       \
            sample_,                                                                      \
            sample_size_,                                                                      \
            camera,                                                                       \
            pose,                                                                       \
            opt,                                                                       \
            weights);                                                                      \
        SWITCH_LOSS_FUNCTIONS
    default:
        return -1;
    }
#undef SWITCH_LOSS_FUNCTION_CASE
}

// Entry point for PnP refinement
int refine_pnp(
    const cv::Mat &correspondences_,
    const size_t *sample_,
    const size_t &sample_size_, 
    const Camera &camera,
    CameraPose *pose,
    const BundleOptions &opt,
    const double *weights) 
{
    switch (camera.model_id) {
#define SWITCH_CAMERA_MODEL_CASE(Model)                                                                                \
    case Model::model_id:                                                                                             \
        return refine_pnp<Model>(                                                                                  \
            correspondences_,                                                                                   \
            sample_,                                                                                  \
            sample_size_,                                                                                  \
            camera,                                                                                   \
            pose,                                                                                   \
            opt,                                                                                   \
            weights);                                     \
        SWITCH_CAMERA_MODELS
#undef SWITCH_CAMERA_MODEL_CASE
    default:
        return -1;
    }
}
} // namespace pose_lib
