cmake_minimum_required(VERSION 3.10)
project(controller)

# 1. Find the Gazebo libraries
find_package(gz-sim8 REQUIRED)
set(GZ_PROP_VER 8)

# 2. Define our plugin (The "Shared Library")
add_library(controller SHARED controller.cpp)

# 3. Link the Gazebo libraries to our code
target_link_libraries(controller
  gz-sim${GZ_PROP_VER}::gz-sim${GZ_PROP_VER}
)