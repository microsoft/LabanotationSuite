#include <aero_labanotation/aerolaban.hh>


namespace aero
{
namespace labanotation
{

//////////////////////////////////////////////////////////////////////////////
AeroLabanInterface::AeroLabanInterface(
    aero::interface::AeroMoveitInterface::Ptr _robot)
{
  robot_ = _robot;

  // current implementation does not support torso rotations
  rotMat_GL2Body_ = Eigen::Matrix3d::Identity();

  // the shoulder_p axis direction of the actual SeedNOID robot
  axis_shoulder_p_[_LEFT]  = Eigen::Vector3d(0, 0.984808,  -0.173648);
  axis_shoulder_p_[_LEFT].normalize();
  axis_shoulder_p_[_RIGHT] = Eigen::Vector3d(0, 0.984808,   0.173648);
  axis_shoulder_p_[_RIGHT].normalize();

  // default setting calculates labanotation for both arms
  arm_ = aero::arm::both_arms;
}

//////////////////////////////////////////////////////////////////////////////
void AeroLabanInterface::analyzeLabanotation(
    const labanotation_msgs::LabanotationTrajectory _traj)
{
  // any joint not set from labanotation will use current/previous value
  // note, waist values will be overwritten to 0 (to be enhanced)
  aero::joint_angle_map init_joint;  // initial values of the current frame
  robot_->getCurrentState(init_joint);  // set current joint vals to init_joint
  init_joint[aero::joint::waist_p] = 0;
  init_joint[aero::joint::waist_r] = 0;
  init_joint[aero::joint::waist_y] = 0;

  // initiate result storage
  keyframe_joints_.clear();
  keyframe_joints_.reserve(_traj.points.size());
  times_from_start_.clear();
  times_from_start_.reserve(_traj.points.size());
  hold_durations_.clear();
  hold_durations_.reserve(_traj.points.size());

  // parse the ROS message
  for (auto pt = _traj.points.begin(); pt != _traj.points.end(); ++pt) {
    // when all the positions are hold, set a hold time for the previous frame
    if (isHoldMotion(*pt)) {
      if (pt == _traj.points.begin())  // not intended
        ROS_WARN("the first frame being a hold frame is buggy!");
      else
        hold_durations_.back()
          = pt->time_from_start.toSec() - times_from_start_.back();
      continue;
    }

    keyframe_joints_.push_back(init_joint);  // initiate w/ the current joints
    calcAnglePose(keyframe_joints_.back(), *pt, arm_);  // calculate/overwrite
    times_from_start_.push_back(pt->time_from_start.toSec());  // copy msg
    hold_durations_.push_back(0);  // will update in next loop if necessary

    // update init_joint w/ the joint angles of the current frame
    init_joint = keyframe_joints_.back();
  }
}

//////////////////////////////////////////////////////////////////////////////
void AeroLabanInterface::execute()
{
  // the current implementation iterates through the keyframes
  // should use sendTrajectory method instead and send all at once (TBD)
  int prev_time = 0;
  for (size_t i = 0; i < keyframe_joints_.size(); ++i) {
    robot_->setRobotStateVariables(keyframe_joints_.at(i));
    robot_->sendModelAngles((times_from_start_.at(i) - prev_time)*1000);
    prev_time = times_from_start_.at(i) + hold_durations_.at(i);
    robot_->waitInterpolation();
    ros::Duration(hold_durations_.at(i)).sleep();
  }
}

//////////////////////////////////////////////////////////////////////////////
bool AeroLabanInterface::isHoldMotion(
    const labanotation_msgs::LabanotationTrajectoryPoint _pt) const
{
  return (_pt.head.hold &&
          _pt.left_elbow.hold && _pt.left_wrist.hold &&
          _pt.right_elbow.hold && _pt.right_wrist.hold);
}

//////////////////////////////////////////////////////////////////////////////
void AeroLabanInterface::calcAnglePose(
    aero::joint_angle_map &_joints,
    const labanotation_msgs::LabanotationTrajectoryPoint _pt,
    const aero::arm _type) const
{
  // we assume that the robot stands at an upright position 

  // calculate head angles
  auto head_v = directionlevel2vec(_pt.head);
  Eigen::Vector3d xyz = rotMat_GL2Body_ * head_v;

  if (!_pt.head.hold) {
    const double n_y
      = angleLimitter(aero::joint::neck_y, atan2(xyz[1], xyz[0]));
    const double n_p
      = angleLimitter(
          aero::joint::neck_p,
          atan2(-xyz[2], calcVal(cos(n_y), xyz[0], sin(n_y), xyz[1])));
    _joints[aero::joint::neck_y] = n_y;
    _joints[aero::joint::neck_p] = n_p;
  }

  // calculate the arm angles
  // both arm has the same kinematics but with different joint limits
  if (_type == aero::arm::larm || _type == aero::arm::both_arms)
    solveArmKinematics(
        _LEFT, _joints,
        directionlevel2vec(_pt.left_elbow),
        directionlevel2vec(_pt.left_wrist),
        _pt.left_elbow.hold, _pt.left_wrist.hold);
  if (_type == aero::arm::rarm || _type == aero::arm::both_arms)
    solveArmKinematics(
        _RIGHT, _joints,
        directionlevel2vec(_pt.right_elbow),
        directionlevel2vec(_pt.right_wrist),
        _pt.right_elbow.hold, _pt.right_wrist.hold);
}

//////////////////////////////////////////////////////////////////////////////
double AeroLabanInterface::angleLimitter(
    const aero::joint _joint, const double _src) const
{
  moveit::core::JointModel::Bounds bounds
    = robot_->kinematic_model->getJointModel(
        aero::joint_map.at(_joint).c_str())->getVariableBounds();

  if (_src > bounds[0].max_position_) {
    ROS_ERROR(
        "Angle Limits (%s) %lf -> %lf",
        aero::joint_map.at(_joint).c_str(), _src, bounds[0].max_position_);
    return bounds[0].max_position_;
  }

  if (_src < bounds[0].min_position_) {
    ROS_ERROR(
        "Angle Limits (%s) %lf -> %lf",
        aero::joint_map.at(_joint).c_str(), _src, bounds[0].min_position_);
    return bounds[0].min_position_;
  }

  return _src;
}

//////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d AeroLabanInterface::directionlevel2vec(
    const labanotation_msgs::LabanotationDirectionSymbol _d) const
{
  Eigen::Vector3d res;
  if (_d.place) {
    res[0] = 0;
    res[1] = 0;
  } else {
    res[0] = fabs(cos(_d.level)) * cos(_d.direction);
    res[1] = fabs(cos(_d.level)) * sin(_d.direction);
  }
  res[2] = sin(_d.level);
  return res; 
}

//////////////////////////////////////////////////////////////////////////////
double AeroLabanInterface::calcVal(
    const double _tx, const double _x, const double _ty, const double _y)
{
  const double abs_val = sqrt(pow(_x, 2) + pow(_y, 2));
  if (fabs(_tx * _x) > fabs(_ty * _y))
    return (_tx * _x > 0 ? abs_val : -abs_val);
  else
    return (_ty * _y > 0 ? abs_val : -abs_val);
}

//////////////////////////////////////////////////////////////////////////////
void AeroLabanInterface::solveArmKinematics(
    const int _arm, aero::joint_angle_map &_joints,
    const Eigen::Vector3d _elbow, const Eigen::Vector3d _wrist,
    const bool _hold_upper, const bool _hold_lower) const
{
  const aero::joint targets[][4] = {
    { aero::joint::l_shoulder_p, aero::joint::l_shoulder_r,
      aero::joint::l_shoulder_y, aero::joint::l_elbow },
    { aero::joint::r_shoulder_p, aero::joint::r_shoulder_r,
      aero::joint::r_shoulder_y, aero::joint::r_elbow },
  };

  // upper arm

  Eigen::Vector3d xyz = rotMat_GL2Body_ * _elbow;
  const double s_p = angleLimitter(targets[_arm][0], atan2(-xyz[0], -xyz[2]));
  const double s_r = angleLimitter(
      targets[_arm][1], atan2(
          xyz[1], calcVal(-sin(s_p), xyz[0], -cos(s_p), xyz[2])));

  // lower arm

  Eigen::Matrix3d intMat;
  intMat = Eigen::AngleAxisd(-s_r, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-s_p, axis_shoulder_p_[_arm]);
  Eigen::Vector3d int_xyz = intMat * (rotMat_GL2Body_ * _wrist);
  const double s_y = angleLimitter(
      targets[_arm][2], atan2(int_xyz[1], int_xyz[0]));
  const double e = angleLimitter(
      targets[_arm][3], atan2(
          calcVal(-cos(s_y), int_xyz[0], -sin(s_y), int_xyz[1]), -int_xyz[2]));

  if (!_hold_upper) {
    _joints[targets[_arm][0]] = s_p;
    _joints[targets[_arm][1]] = s_r;
  }
  if (!_hold_lower) {
    _joints[targets[_arm][2]] = s_y;
    _joints[targets[_arm][3]] = e;
  }
}

}
}
