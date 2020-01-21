/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Kazuhiro Sasabuchi (Microsoft.)
 *  All rights reserved.     
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <memory>

#include <aero_std/AeroMoveitInterface.hh>

#include <labanotation_msgs/LabanotationDirectionSymbol.h>
#include <labanotation_msgs/LabanotationTrajectory.h>
#include <labanotation_msgs/LabanotationTrajectoryPoint.h>


namespace aero
{
namespace labanotation
{

// Labanotation to joint command converter interface for the SeedNOID robot.
// This class should be used to execute labanotation gestures.
// This class solves labanotation using an analytic approach.
// We plan to switch to a MoveIt + sampling approach in the next release.
class AeroLabanInterface
{
protected:
  // @brief Integers (instead of aero::arm) for array indexing.
  const int _LEFT = 0;
  const int _RIGHT = 1;

  // @brief Transformation matrix if there are torso values.
  //        As of current implement, we do not provide any torso value.
  //        Therefore, this is an identity matrix.
  Eigen::Matrix3d rotMat_GL2Body_;

  // @brief shoulder_p rotation axis direction of the SeedNOID.
  //        Define the axis for the correct calculation.
  Eigen::Vector3d axis_shoulder_p_[2];

  // @brief The robot model interface to send commands to the real robot.
  //        Used to send values and get the robot's joint limits.
  aero::interface::AeroMoveitInterface::Ptr robot_;

  // Brief explanation on why below variables are the way they are:
  //
  // Instead of duplicating the frame when a keyframe is a "holding frame"
  // --which is what the msg expects-- we set and store a "holding duration."
  // The original msg structure is designed for "sending gestures"
  // but we often might want to iterate over the non-duplicate keyframes.
  // Thus, we will not store the msg structure as-is but decompose as below.

  // @brief Converted joints from labanotation_msg, set from analyze.
  std::vector<aero::joint_angle_map> keyframe_joints_;

  // @brief Timestamps of each converted joint positions, set from analyze.
  std::vector<double> times_from_start_;

  // @brief Hold duration of each joint positions, set from analyze.
  std::vector<double> hold_durations_;  //

  // @brief The arm that will use labanotation, default is both_arms.
  aero::arm arm_;

public:
  // @brief The constructor.
  // @param[in] _robot The robot model interface provided by aero-ros-pkg.
  explicit AeroLabanInterface(
      aero::interface::AeroMoveitInterface::Ptr _robot);

  // @brief The method to convert ROS messages into joint angles.
  // @param[in] _traj The labanotation ROS message object to compile.
  virtual void analyzeLabanotation(
      const labanotation_msgs::LabanotationTrajectory _traj);

  // @brief The method to send the converted joint angles to the SeedNOID.
  virtual void execute();

  // @brief Set the arm to calculate labanotation (default both).
  inline void setArm(const aero::arm _arm) { arm_ = _arm; };

  // @brief Return the arm that is being calculated.
  // @return The arm that is being calculated.
  inline const aero::arm getArm() { return arm_; };

  // @brief Return the calculated joint angles of the full labanotation.
  // @return A list of list of joint angles.
  inline const std::vector<aero::joint_angle_map> getKeyFrameJoints()
  {
    return keyframe_joints_;
  }

  // @brief Return the starting time of all frames where time of frame0 is 0.
  // @return A list of keyframe starting time.
  inline const std::vector<double> getTimesFromStart()
  {
    return times_from_start_;
  };

  // @brief Return the holding duration of all frames.
  // @return The holding duration of all frames.
  inline const std::vector<double> getHoldDurations()
  {
    return hold_durations_;
  };

  // @brief Define the object pointer.
  typedef std::shared_ptr<AeroLabanInterface> Ptr;

protected:

  // @brief Return whether a frame is holding the pose of the previous frame.
  // @param[in] _pt The labanotation keyframe to check.
  // @return True if is holding the pose of the previous frame.
  bool isHoldMotion(
      const labanotation_msgs::LabanotationTrajectoryPoint _pt) const;

  // @brief Convert a labanotation keyframe into joint angles.
  // @param[in] _joints The result storage initiated w/ the current angles.
  // @param[in] _pt     The labanotation keyframe.
  // @param[in] _arm    The arms to calculate.
  void calcAnglePose(
      aero::joint_angle_map &_joints,
      const labanotation_msgs::LabanotationTrajectoryPoint _pt,
      const aero::arm _arm=aero::arm::both_arms) const;

  // @brief Return angles so that they are within angle limits.
  // @param[in] _joint The joint to check.
  // @param[in] _src   The current angle to set for the joint.
  // @return The angle within the limits.
  double angleLimitter(const aero::joint _joint, const double _src) const;

  // @brief Convert labanotation direction and level to vectors in 3D space.
  // @param[in] _d The labanotation.
  // @return Converted vector in 3D space.
  Eigen::Vector3d directionlevel2vec(
      const labanotation_msgs::LabanotationDirectionSymbol _d) const;

  // @brief Function to determine the appropriate arctan argument value.
  // @param[in] _tx Target x value.
  // @param[in] _x  Reference x value to consider appropriateness.
  // @param[in] _ty Target y value.
  // @param[in] _y  Reference y value to consider appropriateness.
  // @return An appropriate arctan argument value.
  static double calcVal(
      const double _tx, const double _x, const double _ty, const double _y);

  // @brief Function to convert arm directions to SeedNOID joint angles.
  // @param[in] _arm        The arm to calculate.
  // @param[in] _joints     The result storage initiated w/ the current angles.
  // @param[in] _elbow      The upper arm direction in 3D space.
  // @param[in] _wrist      The lower arm direction in 3D space.
  // @param[in] _hold_upper True if upper arm values are same as previous.
  // @param[in] _hold_lower True if lower arm values are same as previous.
  void solveArmKinematics(
      const int _arm, aero::joint_angle_map &_joints,
      const Eigen::Vector3d _elbow, const Eigen::Vector3d _wrist,
      const bool _hold_upper, const bool _hold_lower) const;
};

}
}
