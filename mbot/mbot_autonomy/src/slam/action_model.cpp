#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.01f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_)
    {
        previousPose_ = odometry;
        initialized_ = true;
    }

    float deltaX = odometry.x - previousPose_.x;
    float deltaY = odometry.y - previousPose_.y;
    float deltaTheta = angle_diff(odometry.theta, previousPose_.theta);

    dx_ = deltaX;
    dy_ = deltaY;
    dtheta_ = deltaTheta;

    // Did the robot move?
    bool moved = (deltaX != 0) || (deltaY != 0) || (deltaTheta != 0);

    xStd_ = std::sqrt(k1_ * std::max(std::fabs(dx_), min_dist_));
    yStd_ = std::sqrt(k1_ * std::max(std::fabs(dy_), min_dist_));
    thetaStd_ = std::sqrt(k2_ * std::max(std::fabs(dtheta_), min_theta_));

    utime_ = odometry.utime;
    previousPose_ = odometry;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    mbot_lcm_msgs::particle_t newSample = sample;
    float sampleX = std::normal_distribution<>(dx_, xStd_)(numberGenerator_);
    float sampleY = std::normal_distribution<>(dy_, yStd_)(numberGenerator_);
    float sampleTheta = std::normal_distribution<>(dtheta_, thetaStd_)(numberGenerator_);

    newSample.pose.x += sampleX;
    newSample.pose.y += sampleY;
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleTheta);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}
