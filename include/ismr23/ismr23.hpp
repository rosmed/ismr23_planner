#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace ismr23{

  class psm_planner : public rclcpp::Node{

  private:

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pose;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_description;

    rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr client_fk;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr client_ik;
    rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr client_cp;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client_fjt;
    rclcpp::CallbackGroup::SharedPtr group_cp, group_fjt;

    std::string robot_description;
    moveit_msgs::msg::RobotState seed_state;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    
  public:

    psm_planner( const std::string& name );

    void robot_description_callback( const std_msgs::msg::String& rd );
    void pose_callback( const geometry_msgs::msg::PoseArray& poses );

    void response_callback
    ( rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr handle );
    void feedback_callback
    ( rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr handle,
      const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback );
    void result_callback
    ( const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result );

    
  };
  
}
