CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(VoxelCarving)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# required packages
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(VTK REQUIRED)

# Optional package - ArUco
OPTION(COMPILE_ARUCO "Compile ArUco code" ON)
IF (COMPILE_ARUCO)
    FIND_PACKAGE(aruco)
ENDIF()

# required macros and variables
INCLUDE(${VTK_USE_FILE})

# Main project sources (excluding ArUco code)
FILE(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp 
                  ${CMAKE_SOURCE_DIR}/src/*.h)

# Add ArUco-related sources if COMPILE_ARUCO is ON
IF (COMPILE_ARUCO)
    FILE(GLOB ARUCO_SOURCES ${CMAKE_SOURCE_DIR}/ar_src/*.cpp 
                            ${CMAKE_SOURCE_DIR}/ar_src/*.h)
    SET(SOURCES ${SOURCES} ${ARUCO_SOURCES})
ENDIF()

# Create the executable
ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})

# Link libraries
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${VTK_LIBRARIES}
)

# Build the marker_generator executable only if COMPILE_ARUCO is ON
IF (COMPILE_ARUCO)
    ADD_EXECUTABLE(marker_generator ${CMAKE_SOURCE_DIR}/ar_src/marker_generator.cpp)
    TARGET_LINK_LIBRARIES(marker_generator ${OpenCV_LIBS} ${aruco_LIBS})
ENDIF()
