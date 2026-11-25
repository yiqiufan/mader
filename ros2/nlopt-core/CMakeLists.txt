cmake_minimum_required(VERSION 3.16)
project(mader_nlopt_humble_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MADER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inputs/mader" CACHE PATH "MADER worktree")
set(SEPARATOR_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inputs/separator" CACHE PATH "Locked separator source")
set(GLPK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deps/install" CACHE PATH "GLPK install prefix")

find_package(Eigen3 REQUIRED CONFIG)
find_package(NLopt 2.7.1 EXACT REQUIRED CONFIG)
find_package(CGAL REQUIRED CONFIG)
# CGAL 4.14.2 exports 4.14.1000 in its ConfigVersion file.
if(NOT CGAL_VERSION VERSION_EQUAL "4.14.2")
    message(FATAL_ERROR "Expected CGAL 4.14.2, found ${CGAL_VERSION}")
endif()
find_package(Threads REQUIRED)
find_path(GLPK_INCLUDE_DIR glpk.h PATHS "${GLPK_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(GLPK_LIBRARY glpk PATHS "${GLPK_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

add_library(mader_separator STATIC "${SEPARATOR_SOURCE_DIR}/src/separator_glpk.cpp")
target_include_directories(mader_separator PUBLIC "${SEPARATOR_SOURCE_DIR}/include" PRIVATE "${GLPK_INCLUDE_DIR}")
target_link_libraries(mader_separator PUBLIC Eigen3::Eigen PRIVATE "${GLPK_LIBRARY}")

set(mader "${MADER_SOURCE_DIR}/mader")
add_library(mader_nlopt_core STATIC
    "${mader}/src/solver_nlopt.cpp"
    "${mader}/src/nlopt_utils.cpp"
    "${mader}/src/octopus_search.cpp"
    "${mader}/src/bspline_utils.cpp"
    "${mader}/src/cgal_utils.cpp"
)
target_compile_definitions(mader_nlopt_core PUBLIC USE_GUROBI_FLAG=0)
target_include_directories(mader_nlopt_core PUBLIC "${mader}/include")
target_include_directories(mader_nlopt_core SYSTEM PUBLIC "${MADER_SOURCE_DIR}/thirdparty/DecompROS/DecompUtil/include")
target_link_libraries(mader_nlopt_core PUBLIC Eigen3::Eigen NLopt::nlopt CGAL::CGAL mader_separator Threads::Threads)

add_executable(test_nlopt "${mader}/src/examples/test_nlopt.cpp")
target_link_libraries(test_nlopt PRIVATE mader_nlopt_core)
