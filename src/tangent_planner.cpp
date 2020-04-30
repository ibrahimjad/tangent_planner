/* tangent_planner - BaseLocalPlanner plugin for ROS which uses the tangent bug
 * Copyright (C) 2020 Ibrahim J. Masri
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "tangent_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tangent_planner::TangentPlanner, nav_core::BaseLocalPlanner)

namespace tangent_planner
{
    TangentPlanner::TangentPlanner() : initialized_(false), goal_reached_(false), tf_(nullptr), costmap_ros_(nullptr),
                                       robot_front_(0), robot_back_(0), robot_width_min_(0), robot_width_max_(0),
                                       plan_size_(0), plan_index_(0), kp_(0), kp_i_(0), dreached_(0), dfollowed_(0),
                                       is_new_goal_(true), heuristic_distance_(std::numeric_limits<double>::max())
    {
    }

    TangentPlanner::~TangentPlanner()
    {
        discontinuity_points_.points.clear();
        marker_pub_.publish(discontinuity_points_);
    }

    void TangentPlanner::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        laser_msg_ = msg;
        findDiscontinuities();
        visualizeDiscontinuities();
    }

    void TangentPlanner::initializeSubPub(ros::NodeHandle &n)
    {
        laser_sub_ = n.subscribe("/scan", 1, &TangentPlanner::laserCallback, this);
        marker_pub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    void TangentPlanner::initializeVisualization()
    {
        discontinuity_points_.header.frame_id = "/base_scan";
        discontinuity_points_.ns = "discontinuity_points";
        discontinuity_points_.action = visualization_msgs::Marker::ADD;
        discontinuity_points_.type = visualization_msgs::Marker::POINTS;
        discontinuity_points_.scale.x = discontinuity_points_.scale.y = 0.05;
        discontinuity_points_.color.b = 1.0;
        discontinuity_points_.color.a = 1.0;
        discontinuity_points_.lifetime = ros::Duration();
    }

    void TangentPlanner::visualizeDiscontinuities()
    {
        discontinuity_points_.points.clear();
        geometry_msgs::Point p;

        for (size_t i = 0; i < discontinuities_ranges_.size(); i++)
        {
            p.x = discontinuities_ranges_.at(i) * cos(discontinuities_angles_.at(i));
            p.y = discontinuities_ranges_.at(i) * sin(discontinuities_angles_.at(i));
            p.z = 0;
            discontinuity_points_.points.push_back(p);
        }

        marker_pub_.publish(discontinuity_points_);
    }

    void TangentPlanner::updateFootprint()
    {
        geometry_msgs::Polygon footprint = costmap_ros_->getRobotFootprintPolygon();

        for (auto p : footprint.points)
        {
            robot_front_ = robot_front_ < p.x ? p.x : robot_front_;
            robot_back_ = robot_back_ > p.x ? p.x : robot_back_;
            robot_width_min_ = robot_width_min_ > p.y ? p.y : robot_width_min_;
            robot_width_max_ = robot_width_max_ < p.y ? p.y : robot_width_max_;
        }

        robot_width_ = abs(robot_width_min_) + abs(robot_width_max_);
    }

    void TangentPlanner::updateParameters(ros::NodeHandle &n, const std::string &planner_name)
    {
        const std::string local_costmap_name = costmap_ros_->getName();

        try
        {
            n.setParam("recovery_behavior_enabled", false);
            n.setParam("clearing_rotation_allowed", false);
            n.getParam("controller_frequency", controller_frequency_);
            n.getParam(planner_name + "/max_trans_vel", max_trans_vel_);
            n.getParam(planner_name + "/max_rot_vel", max_rot_vel_);
            n.getParam(planner_name + "/xy_goal_tolerance", xy_goal_tolerance_);
            n.getParam(planner_name + "/yaw_goal_tolerance", yaw_goal_tolerance_);
            n.getParam(local_costmap_name + "/obstacle_range", obstacle_range_);
        }
        catch (const ros::InvalidNameException &e)
        {
            ROS_ERROR("Invalid name: %s", e.what());
        }

        kp_i_ = 1 / (TIME_TO_REACH_MAX_VEL * controller_frequency_);
    }

    bool TangentPlanner::areFramesSetCorrectly()
    {
        const char *global_frame = costmap_ros_->getGlobalFrameID().c_str();

        bool is_correct = false;
        try
        {
            is_correct = tf_->canTransform("odom", global_frame, ros::Time(0), NULL);
            laser_to_base_transform_ = tf_->lookupTransform("odom", "base_scan", ros::Time(0));
        }
        catch (...)
        {
            ROS_ERROR("Error occured while looking up transformation");
        }
        return is_correct;
    }

    void TangentPlanner::initialize(std::string planner_name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (initialized_)
        {
            ROS_INFO("Already initialized!");
        }
        else
        {
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle priv_nh("~");

            initializeSubPub(priv_nh);
            initializeVisualization();

            updateFootprint();
            updateParameters(priv_nh, planner_name);

            if (areFramesSetCorrectly())
            {
                ROS_INFO("The planner is initialized properly!");
                initialized_ = true;
            }
            else
            {
                ROS_ERROR("The tf tree is not set properly between the global frame and the local frame");
                initialized_ = false;
            }
        }
    }

    bool TangentPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_WARN("Planner is not initialized");
            return false;
        }
        else if (plan_size_ != plan.size())
        {
            global_plan_.clear();
            plan_size_ = plan.size();
            goal_reached_ = false;
            geometry_msgs::Pose2D pose2d;
            geometry_msgs::PoseStamped poseStamped;

            for (auto p : plan)
            {
                try
                {
                    tf_->transform<geometry_msgs::PoseStamped>(p, poseStamped, "odom", ros::Duration(0));
                    poseStampedToPose2D(poseStamped, pose2d);
                    global_plan_.push_back(pose2d);
                }
                catch (const tf2::TransformException &e)
                {
                    ROS_ERROR("%s", e.what());
                }
            }
        }
        return true;
    }

    bool TangentPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_WARN("Planner is not initialized");
            return false;
        }

        cmd_ptr_ = &cmd_vel;

        updatePose();
        updateGoal();
        updateState();
        updateController();
        updateVelocity();

        return true;
    }

    void TangentPlanner::updatePose()
    {
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseStamped transformed_pose;
        costmap_ros_->getRobotPose(pose);

        try
        {
            tf_->transform<geometry_msgs::PoseStamped>(pose, transformed_pose, "odom", ros::Duration(0));
            poseStampedToPose2D(transformed_pose, current_pose_);
        }
        catch (const tf2::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
        }
    }

    void TangentPlanner::updateGoal()
    {
        double dx = global_plan_.at(plan_size_ - 1).x - current_pose_.x;
        double dy = global_plan_.at(plan_size_ - 1).y - current_pose_.y;
        goal_distance_ = hypot(dx, dy) - robot_front_;
        goal_angle_ = atan2(dy, dx) - current_pose_.theta;
        normalizeAngle(goal_angle_);
        goal_orientation_ = global_plan_.at(plan_size_ - 1).theta - current_pose_.theta;
        normalizeAngle(goal_orientation_);
        // plan_index_++;
    }

    void TangentPlanner::updateState()
    {
        if (!isInGoalDirection() && !isAtGoal() && isNewGoal())
        {
            state_ = BugState::TurnToGoal;
        }
        else if (isInGoalDirection() && isNewGoal())
        {
            is_new_goal_ = false;
        }
        else if (isAtGoal() && !isAtGoalOrientation())
        {
            state_ = BugState::OrientToGoal;
        }
        else if ((isInGoalDirection() || !foundObstacle()) && isCloseToGoal())
        {
            state_ = BugState::CloseToGoal;
        }
        else if (isAtGoal() && isAtGoalOrientation())
        {
            state_ = BugState::StopAtGoal;
        }
        else if (!foundObstacle())
        {
            state_ = BugState::MotionToGoal;
        }
        else if (foundObstacle())
        {
            state_ = BugState::GoDiscontinuity;
        }
        if (foundLocalMinima() && dfollowed_ >= dreached_)
        {
            state_ = BugState::FollowBoundary;
        }
    }

    void TangentPlanner::updateController()
    {
        if (state_ == BugState::StopAtGoal)
        {
            kp_ = 0;
            angular_velocity_ = 0;
            goal_reached_ = true;
            resetVariables();
        }
        else if (state_ == BugState::TurnToGoal)
        {
            kp_ = 0;
            normalizeAngle(goal_angle_);
            goal_angle_ = abs(goal_angle_) < yaw_goal_tolerance_ ? 0 : goal_angle_;
            angular_velocity_ = 0.5 * goal_angle_;
        }
        else if (state_ == BugState::OrientToGoal)
        {
            kp_ = 0;
            angular_velocity_ = 0.5 * goal_orientation_;
        }
        else if (state_ == BugState::MotionToGoal)
        {
            kp_ = kp_ + kp_i_;
            kp_ = kp_ < 1.0 ? kp_ : 1.0;
            angular_velocity_ = 0.5 * goal_angle_;
        }
        else if (state_ == BugState::GoDiscontinuity)
        {
            selectDiscontinuityPoint();
            kp_ = kp_ + kp_i_;
            kp_ = kp_ < 1.0 ? kp_ : 1.0;
            angular_velocity_ = goal_angle_;
        }
        else if (state_ == BugState::FollowBoundary)
        {
            followBoundaries();
            kp_ = kp_ + kp_i_;
            kp_ = kp_ < 1.0 ? kp_ : 1.0;
            angular_velocity_ = goal_angle_;
        }
        else if (state_ == BugState::CloseToGoal)
        {
            kp_ = kp_ - kp_i_;
            kp_ = kp_ < 0.25 ? 0.25 : kp_;
            angular_velocity_ = goal_angle_;
        }
    }

    void TangentPlanner::updateVelocity()
    {
        cmd_ptr_->angular.z = angular_velocity_;
        cmd_ptr_->angular.z = (cmd_ptr_->angular.z > max_rot_vel_) ? max_rot_vel_ : cmd_ptr_->angular.z;
        cmd_ptr_->linear.x = (max_trans_vel_ - (max_trans_vel_ / max_rot_vel_) * abs(cmd_ptr_->angular.z)) * kp_;
    }

    void TangentPlanner::findDiscontinuities()
    {
        discontinuities_angles_.clear();
        discontinuities_ranges_.clear();
        discontinuities_corrections_.clear();

        double range = 0, next_range = 0;
        double range_diff = 0;
        unsigned int index;

        for (double angle = laser_msg_->angle_min; angle < laser_msg_->angle_max; angle += laser_msg_->angle_increment)
        {
            angleToLaserIndex(angle, index);
            range = laser_msg_->ranges.at(index) - robot_front_;
            index = (++index == laser_msg_->ranges.size()) ? 0 : index;
            next_range = laser_msg_->ranges.at(index) - robot_front_;

            if (std::isnan(range) || range < robot_width_max_ ||
                std::isnan(next_range) || next_range < robot_width_max_)
            {
                continue;
            }

            range_diff = range - next_range;

            if (range_diff > robot_width_)
            {
                discontinuities_ranges_.push_back(next_range);
                discontinuities_angles_.push_back(angle);
                discontinuities_corrections_.push_back(2 * asin(robot_width_min_ / next_range));
            }
            else if (range_diff < -robot_width_)
            {
                discontinuities_ranges_.push_back(range);
                discontinuities_angles_.push_back(angle + laser_msg_->angle_increment);
                discontinuities_corrections_.push_back(2 * asin(robot_width_max_ / range));
            }
        }
    }

    void TangentPlanner::selectDiscontinuityPoint()
    {
        prev_heuristic_distance_ = tangent_selected_ ? heuristic_distance_ : 0;

        tangent_selected_ = false;
        double correction = 0.0;
        double angle = 0.0;
        double obstacle_to_goal_distance = 0.0;
        heuristic_distance_ = std::numeric_limits<double>::max();

        for (size_t i = 0; i < discontinuities_angles_.size(); i++)
        {
            obstacle_to_goal_distance = sqrt(square(discontinuities_ranges_.at(i)) + square(goal_distance_) - 2 * discontinuities_ranges_.at(i) * goal_distance_ * cos(goal_angle_ - discontinuities_angles_.at(i)));
            if (heuristic_distance_ > obstacle_to_goal_distance + discontinuities_ranges_.at(i))
            {
                heuristic_distance_ = obstacle_to_goal_distance + discontinuities_ranges_.at(i);
                angle = discontinuities_angles_.at(i);
                correction = discontinuities_corrections_.at(i);
                tangent_selected_ = true;
            }
        }

        if (tangent_selected_)
        {
            goal_angle_ = angle + correction;
            normalizeAngle(goal_angle_);
            goal_angle_ = abs(goal_angle_) < yaw_goal_tolerance_ ? 0 : goal_angle_;
        }
    }

    void TangentPlanner::followBoundaries()
    {
        unsigned int index;
        bool selected = false;
        double range = 0;
        double temp_angle = 0;
        double right_angle, left_angle;
        double min_distance = std::numeric_limits<double>::max();

        for (double angle = -M_PI_2; angle < M_PI_2; angle += laser_msg_->angle_increment)
        {
            angleToLaserIndex(angle, index);
            range = laser_msg_->ranges.at(index);
            if (std::isnan(range) || std::isinf(range) || range < 0.001)
            {
                continue;
            }
            else if (abs(min_distance - obstacle_range_) >= range - obstacle_range_)
            {
                min_distance = range;
                temp_angle = angle;
                selected = true;
            }
        }
        if (selected)
        {
            dreached_ = sqrt(square(min_distance) + square(goal_distance_) - 2 * min_distance * goal_distance_ * cos(current_pose_.theta - goal_angle_));
            dfollowed_ = (dfollowed_ > dreached_) ? dreached_ : dfollowed_;
            left_angle = temp_angle + M_PI_2;
            right_angle = temp_angle - M_PI_2;
            if ((abs(left_angle + current_pose_.theta) > abs(right_angle + current_pose_.theta)))
            {
                goal_angle_ = right_angle;
            }
            else
            {
                goal_angle_ = left_angle;
            }
            normalizeAngle(goal_angle_);
            goal_angle_ = abs(goal_angle_) < yaw_goal_tolerance_ ? 0 : goal_angle_;
        }
    }

    bool TangentPlanner::foundObstacle()
    {
        unsigned int index;
        angleToLaserIndex(goal_angle_, index);
        double range = laser_msg_->ranges.at(index);

        double start_angle = goal_angle_ + atan2(2 * robot_width_min_, obstacle_range_);
        double end_angle = goal_angle_ + atan2(2 * robot_width_max_, obstacle_range_);

        for (double angle = start_angle; angle <= end_angle; angle += laser_msg_->angle_increment)
        {
            angleToLaserIndex(angle, index);
            range = laser_msg_->ranges.at(index);

            if (std::isnan(range) || std::isinf(range) || range < 0.001)
            {
                continue;
            }
            else if ((range < obstacle_range_ && range <= goal_distance_))
            {
                return true;
            }
        }

        return false;
    }

    void TangentPlanner::resetVariables()
    {
        is_new_goal_ = true;
        tangent_selected_ = false;
        heuristic_distance_ = std::numeric_limits<double>::max();
        dreached_ = std::numeric_limits<double>::max();
        dfollowed_ = std::numeric_limits<double>::max();
    }

    bool TangentPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_WARN("Planner is not initialized");
        }
        else if (goal_reached_)
        {
            ROS_INFO("Goal Reached");
            return true;
        }
        return false;
    }

    bool TangentPlanner::foundLocalMinima()
    {
        return prev_heuristic_distance_ > heuristic_distance_;
    }

    bool TangentPlanner::isCloseToGoal()
    {
        return goal_distance_ < kp_ * max_trans_vel_;
    }

    bool TangentPlanner::isAtGoal()
    {
        return goal_distance_ < xy_goal_tolerance_;
    }

    bool TangentPlanner::isAtGoalOrientation()
    {
        return abs(goal_orientation_) < yaw_goal_tolerance_;
    }

    bool TangentPlanner::isInGoalDirection()
    {
        return abs(goal_angle_) < yaw_goal_tolerance_;
    }

    bool TangentPlanner::isNewGoal()
    {
        return is_new_goal_;
    }

    void TangentPlanner::normalizeAngle(double &angle)
    {
        angle = (angle > M_PI) ? angle - 2 * M_PI : angle;
        angle = (angle < -M_PI) ? angle + 2 * M_PI : angle;
    }

    void TangentPlanner::angleToLaserIndex(double angle, unsigned int &index)
    {
        angle = (angle < 0) ? angle + 2 * M_PI : angle;
        if (angle >= laser_msg_->angle_min && angle <= laser_msg_->angle_max)
        {
            double angle_deg = 180.0 / M_PI * angle;
            double angle_positive = (angle_deg >= 0) ? angle_deg : angle_deg + 360.0;
            index = std::lround(angle_positive);
            index = (index == laser_msg_->ranges.size()) ? 0 : index;
        }
    }

    void TangentPlanner::poseStampedToPose2D(const geometry_msgs::PoseStamped &from, geometry_msgs::Pose2D &to)
    {
        to.x = from.pose.position.x;
        to.y = from.pose.position.y;
        to.theta = tf2::getYaw<geometry_msgs::Quaternion>(from.pose.orientation);
    }

}; // namespace tangent_planner
