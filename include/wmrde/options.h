//options.h
//compile-time options for WMR simulations

#ifndef _WMRDE_OPTIONS_H_
#define _WMRDE_OPTIONS_H_

#include <string>
#include <ros/package.h>

//OPTIONS, set these flags to 1 or 0
#define WMRSIM_USE_QUATERNION 0 //else use Euler angles
#define WMRSIM_DOUBLE_PRECISION 1 //else single (float) precision
//single precision is slower?! lots of "trunction, possible loss of data" warnings
#define WMRSIM_ENABLE_ANIMATION 1 //include WmrAnimation, OGRE dependencies

inline std::string ResourceDir() {
    const std::string path = ros::package::getPath("wmrde")+"/resource/";
    // std::cout << "PATH : << path << std::endl;
    return path;
  // return std::string("/home/rpradeep/Documents/catkin_ws/src/wmrde/resource/");
  // return std::string("/home/pradeepr/catkin_ws/src/wmrde/resource/");
}

inline std::string CADdir() {
    const std::string path = ros::package::getPath("wmrde")+"/CAD/";
    // std::cout << "PATH : << path << std::endl;
    return path;
  // return std::string("/home/rpradeep/Documents/catkin_ws/src/wmrde/CAD/");
  // return std::string("/home/pradeepr/catkin_ws/src/wmrde/CAD/");
}


//don't modify below here

/*
//defines based on options
#if WMRSIM_DOUBLE_PRECISION
typedef double Real;
#else
typedef float Real;
#endif

#if WMRSIM_USE_QUATERNION
#define SIZEORIENT 4
#else
#define SIZEORIENT 3
#endif

//for output only in Debug mode
#ifdef _DEBUG
#define DEBUG_CERR(x) std::cerr << x << std::endl;
#else
#define DEBUG_CERR(x) //do {} while (0)
#endif
*/

#endif  //_WMRDE_OPTIONS_H_
