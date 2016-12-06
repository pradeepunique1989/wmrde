#pragma once

#define DEFAULT_TURN_RADIUS (1000)

class SimInterface
{
private:
    double mTime;
    // This will store the last state of the robot that was received from
    // the simulator
    double mPositionX;
    double mPositionY;
    double mPositionZ;
    double mHeading;
    double mSpeed;

    // This will store the last command received from the Pure Pursuit Controller
    double mSpeedCmd;
    double mTurnRadiusCmd;
public:
    SimInterface()
    {
        mPositionX = 0;
        mPositionY = 0;
        mPositionZ = 0;
        mHeading = 0;
        mSpeed = 0;
        mSpeedCmd = 0;
        mTurnRadiusCmd = DEFAULT_TURN_RADIUS;
    }
    // Stop the compiler generating methods of copy the object
    SimInterface(SimInterface const& copy) = delete;            // Not Implemented
    SimInterface& operator=(SimInterface const& copy) = delete; // Not Implemented
    //These functions are to be used by the simulator (i.e. zoeController)
    inline double getLastSpeedCommand() const
    {
        return mSpeedCmd;
    };
    inline double getLastTurnRadiusCommand() const
    {
        return mTurnRadiusCmd;
    };

    //These functions are to be used by the Pure Pursuit Controller
    inline void setSpeedCmd(const double speedCmd)
    {
        mSpeedCmd = speedCmd;
    }
    inline void setTurnRadiusCmd(const double turnRadiusCmd)
    {
        mTurnRadiusCmd = turnRadiusCmd;
    }

    //These functions are to be used by the planner
    inline double getLastPositionX() const
    {
        return mPositionX;
    };
    inline double getLastPositionY() const
    {
        return mPositionY;
    };
    inline double getLastPositionZ() const
    {
        return mPositionZ;
    };
    inline double getLastHeading() const
    {
        return mHeading;
    };
    inline double getLastSpeed() const
    {
        return mSpeed;
    };

    //These functions are to be used by the zoeController
    inline void setPositionX(const double positionX)
    {
        mPositionX = positionX;
    };
    inline void setPositionY(const double positionY)
    {
        mPositionY = positionY;
    };
    inline void setPositionZ(const double positionZ)
    {
        mPositionZ = positionZ;
    };
    inline void setHeading(const double heading)
    {
        mHeading = heading;
    };
    inline void setSpeed(const double speed)
    {
        mSpeed = speed;
    };

    inline void setTime(const double time)
    {
        mTime = time;
    }
    inline double getTime() const
    {
        return mTime;
    }
};
