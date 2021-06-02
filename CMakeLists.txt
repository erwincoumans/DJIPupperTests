
cmake_minimum_required(VERSION 3.12)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # For LLVM tooling
# set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()



if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0017 NEW)
	cmake_policy(SET CMP0042 NEW)
	cmake_policy(SET CMP0057 NEW)
endif(COMMAND cmake_policy)
    
#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(DJIPupperTests)

IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
    link_libraries(stdc++fs)
ENDIF()



SET(PYTHON_VERSION_DIFF_PHYSICS "" CACHE STRING "Python version pytinydiffsim will use.")
SET(Python_ADDITIONAL_VERSIONS 3 3.6 3.5 3.4 3.3 3.2 3.1 3.0 2.7 2.7.12 2.7.10 2.7.3 )
SET_PROPERTY(CACHE PYTHON_VERSION_DIFF_PHYSICS PROPERTY STRINGS ${Python_ADDITIONAL_VERSIONS})
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build3/cmake ${CMAKE_MODULE_PATH})
OPTION(EXACT_PYTHON_VERSION "Require Python and match PYTHON_VERSION_DIFF_PHYSICS exactly, e.g. 2.7.12" OFF)
IF(EXACT_PYTHON_VERSION)
    set(EXACT_PYTHON_VERSION_FLAG EXACT REQUIRED)
ENDIF(EXACT_PYTHON_VERSION)
# first find the python interpreter
FIND_PACKAGE(PythonInterp ${PYTHON_VERSION_DIFF_PHYSICS} ${EXACT_PYTHON_VERSION_FLAG})
# python library should exactly match that of the interpreter
# the following can result in fatal error if you don't have the right python configuration
FIND_PACKAGE(PythonLibs ${PYTHON_VERSION_STRING} EXACT)


FIND_PACKAGE(PythonLibs)


#add_executable
add_library(pupper_drive SHARED src/PupperDrive.cpp  src/Kinematics.cpp  src/DriveSystem.cpp src/PID.cpp src/Utils.cpp ${TDS_HDRS})

#target_link_libraries(spherical_joint_test_eigen opengl_window tinyxml2  Eigen3::Eigen ${CMAKE_DL_LIBS})
target_compile_definitions(pupper_drive PRIVATE -DUSE_SIM)
target_include_directories(pupper_drive PRIVATE third_party src .pio/libdeps/teensy40/BasicLinearAlgebra ${PYTHON_INCLUDE_DIRS} ${PYTHON_NUMPY_INCLUDE_DIR})


IF (WIN32)
  TARGET_COMPILE_OPTIONS(pupper_drive PUBLIC -bigobj)
  MESSAGE("PYTHON_LIBRARIES")
  MESSAGE(${PYTHON_LIBRARIES})
  GET_FILENAME_COMPONENT(PYTHON_LIBRARY_DIR ${PYTHON_LIBRARIES} DIRECTORY)# CACHE)
  MESSAGE("PYTHON_LIBRARY_DIR ")
  MESSAGE(${PYTHON_LIBRARY_DIR})
  target_link_directories (pupper_drive PUBLIC "${PYTHON_LIBRARY_DIR}")
  TARGET_LINK_LIBRARIES(pupper_drive ${PYTHON_LIBRARIES})
ELSEIF (APPLE)
  SET_TARGET_PROPERTIES(pupper_drive PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
ENDIF ()


