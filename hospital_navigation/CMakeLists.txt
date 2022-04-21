cmake_minimum_required(VERSION 3.5)
project(hospital_navigation)

find_package(ament_cmake_auto REQUIRED)


ament_auto_package(INSTALL_TO_SHARE launch params maps rviz_conf rqt_conf)

