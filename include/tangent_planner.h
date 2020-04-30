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

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/Marker.h>

#ifndef TANGENT_PLANNER_H
#define TANGENT_PLANNER_H

#define square(x) pow(x, 2)

#define TIME_TO_REACH_MAX_VEL 2.0 // sec

namespace tangent_planner
{
class TangentPlanner : public nav_core::BaseLocalPlanner
{
public:
    TangentPlanner();
    ~TangentPlanner();
    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
    bool isGoalReached();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void resetVariables();
    bool areFramesSetCorrectly();

    void findDiscontinuities();
    void visualizeDiscontinuities();
    void selectDiscontinuityPoint();

    void updateFootprint();
    void updateParameters(ros::NodeHandle &n, const std::string &planner_name);

    void updatePose();
    void updateGoal();
    void updateState();
    void updateController();
    void updateVelocity();

    void initializeSubPub(ros::NodeHandle &n);
    void initializeVisualization();

    bool foundObstacle();
    void followBoundaries();

    bool isNewGoal();
    bool isCloseToGoal();
    bool isAtGoal();
    bool isAtGoalOrientation();
    bool isInGoalDirection();
    bool foundLocalMinima();

    void normalizeAngle(double& angle);
    void angleToLaserIndex(double angle, unsigned int &index);
    void poseStampedToPose2D(const geometry_msgs::PoseStamped &from, geometry_msgs::Pose2D &to);

    ros::Publisher marker_pub_;
    ros::Subscriber laser_sub_;
    sensor_msgs::LaserScan::ConstPtr laser_msg_;

    geometry_msgs::Twist *cmd_ptr_;
    geometry_msgs::Pose2D current_pose_;
    std::vector<geometry_msgs::Pose2D> global_plan_;

    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;
    tf2_ros::Buffer *tf_;
    geometry_msgs::TransformStamped laser_to_base_transform_;

    std::size_t plan_size_;
    unsigned int plan_index_;

    bool initialized_;
    bool is_new_goal_;
    bool goal_reached_;
    bool goal_unreachable_;
    bool tangent_selected_;

    double robot_front_, robot_back_, robot_width_min_, robot_width_max_, robot_width_;
    double max_trans_vel_, max_rot_vel_;

    double controller_frequency_;
    double kp_, kp_i_;

    double obstacle_range_;
    double xy_goal_tolerance_, yaw_goal_tolerance_;

    double goal_distance_;
    double goal_angle_;
    double angular_velocity_;
    double goal_orientation_;

    double heuristic_distance_, prev_heuristic_distance_, dfollowed_, dreached_;

    std::vector<double> discontinuities_ranges_;
    std::vector<double> discontinuities_angles_;
    std::vector<double> discontinuities_corrections_;

    visualization_msgs::Marker discontinuity_points_;

    enum BugState
    {
        MotionToGoal,
        FollowBoundary,
        TurnToGoal,
        CloseToGoal,
        OrientToGoal,
        GoDiscontinuity,
        StopAtGoal
    } state_;

};
}; // namespace tangent_planner

#endif
