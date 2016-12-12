#pragma once
#include "Common.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/format.hpp>

class TerrainConfig
{
public:
    TerrainConfig();

    static TerrainConfig loadFromFile(const std::string fullFilePath = "/home/rpradeep/Documents/catkin_ws/src/wmrde/config/terrain.ini")
    // static TerrainConfig loadFromFile(const std::string fullFilePath = "/home/pradeepr/catkin_ws/src/wmrde/config/terrain.ini")
    {
        TerrainConfig config;
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini ( fullFilePath.c_str() , pt );
        config.numOfSegments = pt.get<int> ("Terrain.numOfSegments");
        for(int i=0;i<config.numOfSegments;i++)
        {
            std::stringstream ss;
            ss << "Segment"<<i<<".length";
            config.segmentLength.push_back(pt.get<double>(ss.str().c_str()));

            std::stringstream ss2;
            ss2 << "Segment"<<i<<".slope";
            config.segmentSlope.push_back(pt.get<double>(ss2.str().c_str()));
        }
        return config;
    }
    int numOfSegments;
    std::vector<double> segmentLength;
    std::vector<double> segmentSlope;

    double getHeightAt(const double positionX) const;
    double getSlopeAt(const double positionX) const;
};
