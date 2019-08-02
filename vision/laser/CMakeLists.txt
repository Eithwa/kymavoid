add_executable(laser main.cpp laser.cpp nodehandle.cpp)
target_link_libraries(laser ${catkin_LIBRARIES})
add_dependencies(laser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(laser ${OpenCV_LIBRARIES})
