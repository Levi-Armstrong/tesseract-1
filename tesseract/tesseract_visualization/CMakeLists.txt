cmake_minimum_required(VERSION 3.5.0)
project(tesseract_visualization VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(ignition-rendering3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE tesseract::tesseract_collision_fcl tesseract::tesseract_collision_bullet tesseract::tesseract_common)
tesseract_target_compile_options(${PROJECT_NAME} INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME})
tesseract_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

add_library(tesseract_viewer SHARED src/tesseract_viewer.cpp src/glut_window.cpp)
target_link_directories(tesseract_viewer PUBLIC )
target_link_libraries(tesseract_viewer PUBLIC tesseract::tesseract_environment_core tesseract::tesseract_common OpenGL::OpenGL GLUT::GLUT GLEW::GLEW ${IGNITION-RENDERING_LIBRARIES})
tesseract_target_compile_options(tesseract_viewer PUBLIC)
tesseract_clang_tidy(tesseract_viewer)
tesseract_code_coverage(tesseract_viewer ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(tesseract_viewer PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(tesseract_viewer SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

add_executable(demo src/demo.cpp )
target_link_libraries(demo PUBLIC tesseract_viewer tesseract::tesseract_support tesseract::tesseract_urdf tesseract::tesseract_environment_kdl)
tesseract_target_compile_options(demo PUBLIC)
tesseract_clang_tidy(demo)
tesseract_code_coverage(demo ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(demo SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

tesseract_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
