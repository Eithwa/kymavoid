add_executable(blackItem image_converter.cpp blackItem.cpp)
target_link_libraries(blackItem ${catkin_LIBRARIES})
add_dependencies(blackItem ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(blackItem ${OpenCV_LIBRARIES})
