cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 17)
project(belief_search)

include_directories(${PROJECT_SOURCE_DIR}/include/)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SRC_FILES
    "src/node.cpp"
    "src/utils.cpp"
    "src/model.cpp"
    "src/gridworld_triangles.cpp"
    "src/workspace.cpp"
    "src/planner.cpp"
    )

configure_file("play.py" "play.py" COPYONLY)
configure_file("Model.py" "Model.py" COPYONLY)
configure_file("Node.py" "Node.py" COPYONLY)
configure_file("GridWorldDirections.py" "GridWorldDirections.py" COPYONLY)



add_executable(run_comparison "run_comparison.cpp" ${SRC_FILES})
target_link_libraries(run_comparison Eigen3::Eigen)

add_executable(run_single "run_single.cpp" ${SRC_FILES})
target_link_libraries(run_single Eigen3::Eigen)



