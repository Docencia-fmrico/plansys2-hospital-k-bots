cmake_minimum_required(VERSION 3.5)
project(plansys2_hospital)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    nav2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
)

add_executable(move_directly_action_node src/move_directly_action_node.cpp)
ament_target_dependencies(move_directly_action_node ${dependencies})

add_executable(move_through_action_node src/move_through_action_node.cpp)
ament_target_dependencies(move_through_action_node ${dependencies})

add_executable(close_door_action_node src/close_door_action_node.cpp)
ament_target_dependencies(close_door_action_node ${dependencies})

add_executable(open_door_action_node src/open_door_action_node.cpp)
ament_target_dependencies(open_door_action_node ${dependencies})

add_executable(drop_object_action_node src/drop_object_action_node.cpp)
ament_target_dependencies(drop_object_action_node ${dependencies})

add_executable(pick_object_action_node src/pick_object_action_node.cpp)
ament_target_dependencies(pick_object_action_node ${dependencies})

add_executable(patrolling_controller_node src/patrolling_controller_node.cpp)
ament_target_dependencies(patrolling_controller_node ${dependencies})

install(DIRECTORY launch pddl params DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_directly_action_node
  move_through_action_node
  close_door_action_node
  open_door_action_node
  drop_object_action_node
  pick_object_action_node
  patrolling_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
