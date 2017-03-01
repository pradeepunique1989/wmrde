#pragma once

#include <wmrde/Common.h>
#include <wmrde/test.h>
#include <wmrde/demo/terrains.h>
#include <wmrde/demo/models.h>
#include <wmrde/dynamics.h>
#include <wmrde/SimInterface.h>
#include <wmrde/TerrainConfig.h>

#include <csignal>

#include <chrono>
#include <boost/thread.hpp>
#include <thread>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

// Local function prototypes
void simulatorThread();
void init();
void cleanup();

// Variables for calculating speed of robot
Eigen::Vector3d oldPos(0,0,0);
// std::ofstream of("pos.txt");
// void updateSimInterface(const double x,const double y,const double z,const double heading,const double dt,const double currentTime);

void sub_initTrackContactGeom(const WmrModel& mdl, TrackContactGeom* contacts);
// Global objects go here

WmrAnimation anim;
SimInterface simInterface;
TerrainConfig terrainInfo = TerrainConfig::loadFromFile(ros::package::getPath("wmrde")+"/config/terrain.ini");
