// A ROS node to execute labanotation ROS messages as robot commands.

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <aero_labanotation/aerolaban.hh>
#include <labanotation_msgs/LabanotationAction.h>


// @brief A class to execute labanotation ROS messages as robot commands.
class AeroLabanotationAction
{
private:
  ros::NodeHandle nh_;
  aero::interface::AeroMoveitInterface::Ptr robot_;
  aero::labanotation::AeroLabanInterface::Ptr labanInterface_;
  actionlib::SimpleActionServer<labanotation_msgs::LabanotationAction> as_;
  labanotation_msgs::LabanotationResult result_;

public:
  // @brief The constructor.
  // @param[in] _nh ROS node handler.
  explicit AeroLabanotationAction(ros::NodeHandle &_nh) :
    as_(_nh, "/aero_labanotation/labanotation_action",
        boost::bind(&AeroLabanotationAction::executeCB, this, _1),
        false),
    nh_(_nh)
  {
    // initiate the SeedNOID class object
    robot_.reset(new aero::interface::AeroMoveitInterface(nh_));

    // initiate the interface to convert labanotation messages to commands
    labanInterface_.reset(new aero::labanotation::AeroLabanInterface(robot_));

    as_.start();  // start the command server
  }

  // @brief A callback to handle incoming labanotation ROS messages.
  // @param[in] _goal A ROS msg containing labanotation keyframes.
  void executeCB(const labanotation_msgs::LabanotationGoalConstPtr &_goal)
  {
    // store the message to command
    labanInterface_->analyzeLabanotation(_goal->trajectory);

    labanInterface_->execute();  // execute the command

    // tell the command requester that the execution was successful
    result_.error_code = labanotation_msgs::LabanotationResult::SUCCESSFUL;
    result_.error_string = "";
    as_.setSucceeded(result_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_labanotation");
  ros::NodeHandle nh;

  AeroLabanotationAction server(nh);  // create the server
  ros::spin();

  return 0;
}

