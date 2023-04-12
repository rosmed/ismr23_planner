#include <ismr23/ismr23.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/macros/console_colors.h>
#include <boost/tokenizer.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);
  std::shared_ptr<ismr23::psm_planner> planner=std::make_shared<ismr23::psm_planner>("psm_planner");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 10 );
  executor.add_node(planner);

  executor.spin();
  
  rclcpp::shutdown();
  
  return 0;
  
}
