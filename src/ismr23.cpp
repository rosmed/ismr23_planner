#include <ismr23/ismr23.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

using namespace ismr23;
using namespace std::chrono_literals;
using namespace std::placeholders;

psm_planner::psm_planner( const std::string& name ) :
  rclcpp::Node(name){


  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscribing to pose topic");
  sub_pose = create_subscription<geometry_msgs::msg::PoseArray>( "pose_array", 10, 
	     std::bind(&psm_planner::pose_callback, this, std::placeholders::_1 ) );
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscribing to description topic");
  sub_description = create_subscription<std_msgs::msg::String>( "robot_description", 10, 
		    std::bind(&psm_planner::robot_description_callback, this,
			      std::placeholders::_1 ) );

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating ID client");
  client_ik = create_client<moveit_msgs::srv::GetPositionIK>( "/compute_ik" );
  client_fk = create_client<moveit_msgs::srv::GetPositionFK>( "/compute_fk" );
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating planning client");
  group_cp = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  client_cp = create_client<moveit_msgs::srv::GetCartesianPath>( "/compute_cartesian_path",
  								 rmw_qos_profile_t(),
  								 group_cp );

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating control client");
  group_fjt = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  client_fjt =
    rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>( this,
									       "dvrk_trajectory_controller/follow_joint_trajectory",
									       group_fjt );
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for robot action service..");
  //client_fjt->wait_for_action_server();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving on.");

  seed_state.joint_state.name.push_back("outer_yaw");
  seed_state.joint_state.name.push_back("outer_pitch_1");
  seed_state.joint_state.name.push_back("outer_pitch_3");
  seed_state.joint_state.name.push_back("outer_pitch_5");

  seed_state.joint_state.name.push_back("outer_insertion");
  
  seed_state.joint_state.name.push_back("outer_roll");
  seed_state.joint_state.name.push_back("outer_wrist_pitch");
  seed_state.joint_state.name.push_back("outer_wrist_yaw");

  seed_state.joint_state.name.push_back("jaw");
  seed_state.joint_state.name.push_back("jaw_mimic_1");
  seed_state.joint_state.name.push_back("jaw_mimic_2");

  seed_state.joint_state.name.push_back("outer_pitch_4");
  seed_state.joint_state.name.push_back("outer_pitch_2");
  
  
  for( std::size_t i=0; i<seed_state.joint_state.name.size(); i++ )
    seed_state.joint_state.position.push_back(0.0);
  seed_state.joint_state.header.frame_id = "world";
}

void psm_planner::robot_description_callback( const std_msgs::msg::String& rd ){
  robot_description = rd.data;
  std::cout << robot_description << std::endl;
}

void psm_planner::pose_callback( const geometry_msgs::msg::PoseArray& poses ){

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received poses" );

  // get current position from tf and add to waypoints
  
  auto request = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
  request->header = poses.header;
  request->header.frame_id = "world";
  request->waypoints = poses.poses;
  request->start_state = seed_state;
  request->group_name = "psm1_arm";
  request->link_name = "PSM1_tool_tip_link";
  request->max_step = 0.010000;
  request->jump_threshold = 0.0;
  request->avoid_collisions = false;
  /*
  geometry_msgs::msg::Pose Rt;
  Rt.position.x = -0.26;
  Rt.position.y = 0.0;
  Rt.position.z = 0.49;
  
  Rt.orientation.x =  0.70710678118654752440;
  Rt.orientation.y = -0.70710678118654752440;
  Rt.orientation.z = 0.0;
  Rt.orientation.w = 0.0;
  request->waypoints.push_back(Rt);
  */

  for( size_t i=0; i<poses.poses.size(); i++ ){
    /*
    tf2::Transform Rt;
    tf2::convert(request->waypoints[i], Rt);
    tf2::Transform Rotx( tf2::Quaternion(M_SQRT2/2, 0.0, 0.0, M_SQRT2/2), tf2::Vector3(0, 0, 0) );
    tf2::Transform Rtrot = Rt*Rotx;
    tf2::toMsg(Rtrot, request->waypoints[i]);
    */
    request->waypoints[i].orientation.x = 0.70710678118654752440;
    request->waypoints[i].orientation.y = -0.70710678118654752440;
    request->waypoints[i].orientation.z = 0;
    request->waypoints[i].orientation.w = 0;

  }
  // wait for CP service
  while(!client_cp->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
		   "Interrupted while waiting for the Cartesian path service. Exiting.");
      return;
    }
    RCLCPP_INFO
      (rclcpp::get_logger("rclcpp"), "Cartesian path service not available, waiting again...");
  }

  
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending planning request");
  auto result = client_cp->async_send_request(request);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent planning request");
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning call success" );

  switch( result.get()->error_code.val ){
  case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cartesian path success");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fraction %f", result.get()->fraction);
    }
    break;
  case moveit_msgs::msg::MoveItErrorCodes::FAILURE:
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path failure");
    }
    break;
  case moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED:
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planning failed");
    }
    break;
  case moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN:
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid motion plan");
    }
    break;
  case moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Plan invalidated by environment");
    }
    break;
  case moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME:
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid group name");
    }
    break;
  case moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE:
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid robot state");
    }
    break;
  case moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME:
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid link name");
    }
    break;
  default:
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error");
    }
    break;
  }

  /*
  if( result.get()->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS ){

    control_msgs::action::FollowJointTrajectory::Goal resection;
    resection.trajectory = result.get()->solution.joint_trajectory;

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&psm_planner::response_callback, this, _1);
    options.feedback_callback = std::bind(&psm_planner::feedback_callback, this, _1, _2);
    options.result_callback = std::bind(&psm_planner::result_callback, this, _1);

    auto future = client_fjt->async_send_goal( resection, options );
    auto result = client_fjt->async_get_result(future.get());
    result.get();
    std::cout << "done" << std::endl;
    
  }
  */
  
}

void psm_planner::response_callback
( rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr )
{ std::cout << "client response" << std::endl; }

void psm_planner::feedback_callback
( rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr ,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> )
{ std::cout << "feedback " << std::endl; }

void psm_planner::result_callback
( const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult&  ){
  std::cout << "result " << std::endl;
}

/*
    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = "psm1_arm";
    request->ik_request.robot_state = seed_state;

    // wait for IK service
    while(!client_ik->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
		     "Interrupted while waiting for the IK service. Exiting.");
	return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client_ik->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
	rclcpp::FutureReturnCode::SUCCESS){
      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IK call success" );
      
      switch( result.get()->error_code.val ){
      case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
	{
	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IK solution success" );
	  
	}
	break;
      case moveit_msgs::msg::MoveItErrorCodes::FAILURE:
	{
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "IK failure");
	}
	break;
      case moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION:
	{
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No IK solution");
	}
	break;
      default:
	{
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknon IK error code");
	  break;
	}
      }
    }
    else
      { RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "IK call failed"); }
    
*/

  /*
  auto fk_req = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
  fk_req->header.frame_id = "world";
  fk_req->header.stamp = now();
  fk_req->fk_link_names.push_back( "PSM1_tool_tip_link" );
  fk_req->robot_state = seed_state;
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending FK request");
    auto fk_result = client_fk->async_send_request(fk_req);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent FK request");
    std::cout << fk_result.get()->error_code.val << std::endl;
  }
  */
  
