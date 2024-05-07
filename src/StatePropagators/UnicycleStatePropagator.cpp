/* Author: Qi Heng */

#include "UnicycleStatePropagator.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;




DynUnicycleControlSpace::DynUnicycleControlSpace(const oc::SpaceInformationPtr &si, double process_noise) : oc::StatePropagator(si) {

    duration_ = si->getPropagationStepSize();

    forward_acceleration_bounds_.push_back(-0.12);
    forward_acceleration_bounds_.push_back(0.12);
    turning_rate_bounds_.push_back(-0.5); //-2.0
    turning_rate_bounds_.push_back(0.5); //2.0
    surge_bounds_.push_back(0.1);
    surge_bounds_.push_back(5.0);
    //=========================================================================
    // Open loop system definition
    //=========================================================================
    A_ol_ << 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, 0.0;

    B_ol_ << 0.0, 0.0,
             1.0, 0.0,
             0.0, 0.0,
             0.0, 1.0;
    //=========================================================================
    // PD controller coefficients (from LQR)
    //=========================================================================
    // K_.resize(2, 4);
    K_(0, 0) = 0.0316;
    K_(0, 1) = 0.3054;
    K_(0, 2) = 0.0;
    K_(0, 3) = 0.0;
    K_(1, 0) = 0.0;
    K_(1, 1) = 0.0;
    K_(1, 2) = 0.0316;
    K_(1, 3) = 0.3054;

    controller_parameters_.push_back(0.0316);
    controller_parameters_.push_back(0.3054);
    controller_parameters_.push_back(0.0);
    controller_parameters_.push_back(0.0);
    controller_parameters_.push_back(0.0);
    controller_parameters_.push_back(0.0);
    controller_parameters_.push_back(0.0316);
    controller_parameters_.push_back(0.3054);
    //=========================================================================
    // Close loop system definition
    //=========================================================================
    // A_cl_.resize(4, 4);
    A_cl_ = A_ol_ - B_ol_ * K_;

    A_cl_d_.resize(4, 4);
    A_cl_d_ = Eigen::MatrixXd::Identity(4, 4) - A_cl_ * duration_;

    A_cl_d_ = A_cl_d_.inverse().eval();

    // set a 2 by 2 matrix for sigma uncertainty propagation
    A_cl_d_22_.resize(2, 2);
    A_cl_d_22_(0, 0) = A_cl_d_(0,0); //0.7; //A_cl_d_(0, 0);
    A_cl_d_22_(0, 1) = A_cl_d_(0, 2);
    A_cl_d_22_(1, 0) = A_cl_d_(2, 0);
    A_cl_d_22_(1, 1) = A_cl_d_(2,2); //0.7; //A_cl_d_(2, 2);

    A_BK_(0,0) = 0.7; //A_cl_d_22_(0,0);
    A_BK_(1,1) = 0.7; //A_cl_d_22_(1,1);
    A_BK_(0,1) = 0.0;
    A_BK_(1,0) = 0.0;

    double processNoise = process_noise; //process noise is 0.0 for scenario
    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);

    Eigen::MatrixXd Ahat(6,6);
    Ahat.topLeftCorner(4,4) = A_ol_;
    Ahat.topRightCorner(4,2) = B_ol_;
    Ahat.bottomLeftCorner(2,4) = Eigen::MatrixXd::Zero(2,4);
    Ahat.bottomRightCorner(2,2) = Eigen::MatrixXd::Zero(2,2);

}

void saturate(double &value, const double min_value, const double max_value) {
    if(value < min_value) value = min_value;
    if(value > max_value) value = max_value;
}

double wraps(double angle) {
    angle = fmod(angle, 2 * M_PI);
    if (angle > M_PI) angle -= 2 * M_PI;
    return angle;
}

void DynUnicycleControlSpace::propagate(const ob::State *start, const oc::Control* control, const double duration, ob::State *result) const {
    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    double x_pose, y_pose, yaw, surge, Pxx_init, Pyy_init;
    
    start_css = start->as<ob::CompoundStateSpace::StateType>();

    x_pose = start_css->as<R2BeliefSpace::StateType>(0)->getX();
    y_pose = start_css->as<R2BeliefSpace::StateType>(0)->getY();
    yaw = start_css->as<ob::SO2StateSpace::StateType>(1)->value;
    surge = start_css->as<ob::RealVectorStateSpace::StateType>(2)->values[0];

    cos_y = cos(yaw);
    sin_y = sin(yaw);
    // std::cout << i++ << std::endl;
    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================
    double x_pose_reference, y_pose_reference, yaw_reference, surge_reference;

    control_css = control->as<oc::CompoundControlSpace::ControlType>();
    const oc::RealVectorControlSpace::ControlType *control_css_rvs_pose = control_css->as<oc::RealVectorControlSpace::ControlType>(0);

    x_pose_reference = control_css_rvs_pose->values[0];
    y_pose_reference = control_css_rvs_pose->values[1];
    yaw_reference = control_css->as<oc::RealVectorControlSpace::ControlType>(1)->values[0];
    surge_reference = control_css->as<oc::RealVectorControlSpace::ControlType>(2)->values[0];

    // std::cout << x_pose_reference << " " << y_pose_reference << std::endl;

    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y))) with PD controller
    //=========================================================================
    double u_0 = controller_parameters_[0] * (x_pose_reference - x_pose) + controller_parameters_[1] * (surge_reference * cos(yaw_reference) - surge * cos_y);
    double u_1 = controller_parameters_[6] * (y_pose_reference - y_pose) + controller_parameters_[7] * (surge_reference * sin(yaw_reference) - surge * sin_y);
    // std::cout << i++ << std::endl;
    //=========================================================================
    // Get dot(v) and dot(yaw) from dot(dot(x)) dot(dot(y))
    //=========================================================================
    double u_bar_0 = cos_y * u_0 + sin_y * u_1;
    double u_bar_1 = (-sin_y * u_0 + cos_y * u_1) / surge;

    //=========================================================================
    // Bound controller outputs (dot(v) and dot(yaw))
    //=========================================================================
    saturate(u_bar_0, forward_acceleration_bounds_[0], forward_acceleration_bounds_[1]);
    saturate(u_bar_1, turning_rate_bounds_[0], turning_rate_bounds_[1]);
    //=========================================================================
    // Bound surge
    //=========================================================================
    double surge_final = surge + duration * u_bar_0;
    saturate(surge_final, surge_bounds_[0], surge_bounds_[1]);
    
    //=========================================================================
    // Propagate mean
    //=========================================================================
    ob::CompoundStateSpace::StateType *result_css = result->as<ob::CompoundStateSpace::StateType>();
    R2BeliefSpace::StateType *result_css_rvs_pose = result_css->as<R2BeliefSpace::StateType>(0);

    result_css_rvs_pose->setX(x_pose + duration * cos_y * surge_final);
    result_css_rvs_pose->setY(y_pose + duration * sin_y * surge_final);
    result_css->as<ob::SO2StateSpace::StateType>(1)->value = wraps(yaw + duration * u_bar_1);
    result_css->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = surge_final;
    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================
    ob::RealVectorStateSpace::StateType *result_css_rvs_cov = result_css->as<ob::RealVectorStateSpace::StateType>(3);

    Eigen::Matrix2d sigma_from = start_css->as<R2BeliefSpace::StateType>(0)->getSigma();
    Eigen::Matrix2d lambda_from = start_css->as<R2BeliefSpace::StateType>(0)->getLambda();
    Eigen::Matrix2d sigma_pred = A_cl_d_22_*sigma_from*A_cl_d_22_ + Q; //F*sigma_from*F + Q; //
    Mat K, lambda_pred;

    if (result_css_rvs_pose->getY()< -40.0 && sigma_pred(0,0) < 5.8){
        Mat R = 0.05*Eigen::Matrix2d::Identity();
        Eigen::Matrix2d S = (H * sigma_pred * H.transpose())+ R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred = A_BK_*lambda_from*A_BK_;
    }
    else{
        K = Eigen::Matrix2d::Zero(dimensions_, dimensions_);
        lambda_pred = lambda_from;
    }
    Eigen::Matrix2d sigma_to = (I - (K*H)) * sigma_pred;
    Eigen::Matrix2d lambda_to = lambda_pred + K*H*sigma_pred;

    result_css_rvs_pose->setSigma(sigma_to);
    result_css_rvs_pose->setLambda(lambda_to);
}


bool DynUnicycleControlSpace::canPropagateBackward(void) const
{
    return false;
}