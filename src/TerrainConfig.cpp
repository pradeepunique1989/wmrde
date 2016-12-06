#include <wmrde/TerrainConfig.h>

#define DEG_TO_RAD (M_PI/180)
TerrainConfig::TerrainConfig()
{
}

double TerrainConfig::getHeightAt(const double positionX) const
{
    double returnVal = 0;
    double cumSum = 0;
    double lastHeight = 0;
    for(int i = 0; i<numOfSegments; i++)
    {
        cumSum += segmentLength[i];
        const double heightChange = std::tan(DEG_TO_RAD*segmentSlope[i]) * segmentLength[i]; 
        if ( positionX < cumSum )
        {
            const double ratio = 1 - ( (cumSum - positionX) / segmentLength[i] );
            returnVal = lastHeight + ratio * heightChange; 
            return returnVal;
        }
        lastHeight += heightChange;
    }
    returnVal = lastHeight + (positionX-cumSum)*std::tan(DEG_TO_RAD*segmentSlope.back());
    return returnVal;
}

double TerrainConfig::getSlopeAt(const double positionX) const
{
    double returnVal = 0;
    double cumSum = 0;
    for(int i = 0; i<numOfSegments; i++)
    {
        cumSum += segmentLength[i];
        const double heightChange = std::tan(DEG_TO_RAD*segmentSlope[i]) * segmentLength[i]; 
        if ( positionX < cumSum )
        {
            returnVal = segmentSlope[i]; 
            return returnVal;
        }
    }
    returnVal = segmentSlope.back();
    return returnVal;
}
