cmake_minimum_required(VERSION 3.16)
project(mader_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mader_msgs REQUIRED)
find_package(mader_handoff_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snapstack_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(NLopt 2.7.1 EXACT REQUIRED CONFIG)
find_package(CGAL REQUIRED CONFIG)

set(MADER_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH "Locked MADER source")
set(SEPARATOR_SOURCE_DIR "${MADER_SOURCE_DIR}/submodules/separator" CACHE PATH "Locked separator source")
set(NLOPT_CORE_RECIPE "${CMAKE_CURRENT_LIST_DIR}/../nlopt-core" CACHE PATH "MADER core recipe")

foreach(input MADER_SOURCE_DIR SEPARATOR_SOURCE_DIR GLPK_ROOT NLOPT_CORE_RECIPE)
  if(NOT IS_DIRECTORY "${${input}}")
    message(FATAL_ERROR "Provide an existing source or dependency directory for ${input}")
  endif()
endforeach()
add_subdirectory("${NLOPT_CORE_RECIPE}" "${CMAKE_CURRENT_BINARY_DIR}/nlopt-core")
set_property(TARGET mader_separator mader_nlopt_core test_nlopt PROPERTY CXX_STANDARD 17)

add_executable(mader_handoff_node mader_node.cpp "${MADER_SOURCE_DIR}/mader/src/mader.cpp")
ament_target_dependencies(mader_handoff_node geometry_msgs mader_msgs mader_handoff_interfaces rclcpp snapstack_msgs tf2)
target_link_libraries(mader_handoff_node mader_nlopt_core)
set_target_properties(mader_handoff_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS mader_handoff_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS perfect_tracker.py DESTINATION lib/${PROJECT_NAME})
install(FILES mader.yaml LICENSE DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# Preserve the SONAME for deployment on the locked Humble base image.
install(FILES "$<TARGET_FILE:NLopt::nlopt>"
  DESTINATION lib RENAME "$<TARGET_SONAME_FILE_NAME:NLopt::nlopt>")
install(FILES "$<TARGET_FILE:CGAL::CGAL>"
  DESTINATION lib RENAME "$<TARGET_SONAME_FILE_NAME:CGAL::CGAL>")
ament_package()
