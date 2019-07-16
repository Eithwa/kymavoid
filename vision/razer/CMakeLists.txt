add_executable(razer main.cpp razer.cpp nodehandle.cpp)
target_link_libraries(razer ${catkin_LIBRARIES})
add_dependencies(razer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(razer ${OpenCV_LIBRARIES})
