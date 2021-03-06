cmake_minimum_required(VERSION 2.8.3)
 
project(wmrde)
 
if(EXISTS "/usr/local/lib/OGRE/cmake")

  set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
  set(OGRE_SAMPLES_INCLUDEPATH "/usr/local/share/OGRE/samples/Common/include/") # We could just *assume* that developers uses this basepath : /usr/local

elseif(EXISTS "/usr/lib/OGRE/cmake")

  set(CMAKE_MODULE_PATH "/usr/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
  set(OGRE_SAMPLES_INCLUDEPATH "/usr/share/OGRE/samples/Common/include/") # Otherwise, this one

else ()
  message(SEND_ERROR "Failed to find module path.")
endif(EXISTS "/usr/local/lib/OGRE/cmake")
 
if (CMAKE_BUILD_TYPE STREQUAL "")
  # CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
  # differentiation between debug and release builds.
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()
 
set(CMAKE_DEBUG_POSTFIX "_d")
 
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")
 
find_package(OGRE REQUIRED)
find_package(OIS REQUIRED)
 
if(NOT OIS_FOUND)
	message(SEND_ERROR "Failed to find OIS.")
endif()
 
# Find Boost
if (NOT OGRE_BUILD_PLATFORM_IPHONE)
	set(Boost_USE_STATIC_LIBS ${OGRE_STATIC})
	set(Boost_ADDITIONAL_VERSIONS "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
	# Components that need linking (NB does not include header-only components like bind)
	set(OGRE_BOOST_COMPONENTS thread date_time)
	find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
	if (NOT Boost_FOUND)
		# Try again with the other type of libs
		set(Boost_USE_STATIC_LIBS NOT ${Boost_USE_STATIC_LIBS})
		find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
	endif()
	find_package(Boost QUIET)
 
	# Set up referencing of Boost
	include_directories(${Boost_INCLUDE_DIR})
	add_definitions(-DBOOST_ALL_NO_LIB)
	set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Boost_LIBRARIES})
endif()

file(GLOB_RECURSE HDRS
	../include/*.h
)
 
file(GLOB_RECURSE SRCS
	./*.cpp
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}") #added by Neal

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    nav_msgs
    tf
    tf_conversions
    roslib
    )
catkin_package()

include_directories( 
  # ../include #added by Neal
  ./include #added by Neal
  ${OIS_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_SAMPLES_INCLUDEPATH}
  ${OGRE_Overlay_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS} #added by Neal
  /usr/local/include #added by Neal, TODO /ode necessary?
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)
 
# add_executable(OgreApp WIN32 ${HDRS} ${SRCS})
add_executable(OgreApp ${HDRS} ${SRCS})
 
set_target_properties(OgreApp
    PROPERTIES DEBUG_POSTFIX _d
    )
# set_target_properties(OgreApp
#     PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION}
#     )
 
target_link_libraries(OgreApp ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${OGRE_Overlay_LIBRARIES} 
    ${catkin_LIBRARIES}
    ) #added by Neal
message(WARNING "catkin package bin dest : ${CATKIN_PACKAGE_BIN_DESTINATION}")
 
file(MAKE_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION}/dist/bin)
file(MAKE_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION}/dist/media)
 
# set(EXECUTABLE_OUTPUT_PATH ${CATKIN_PACKAGE_BIN_DESTINATION}/dist/bin)
install(TARGETS OgreApp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/bin
    CONFIGURATIONS Release RelWithDebInfo Debug
    )

install(DIRECTORY ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/dist/media
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    CONFIGURATIONS Release RelWithDebInfo Debug
)

install(FILES ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/dist/bin/plugins.cfg
    ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/dist/bin/resources.cfg
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/bin
    CONFIGURATIONS Release RelWithDebInfo Debug
)

# install(FILES ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/OgreApp
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/bin
#     CONFIGURATIONS Release RelWithDebInfo Debug
# )
