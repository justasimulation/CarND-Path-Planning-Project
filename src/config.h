#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#include "utils.h"

/***
 * Configurable variables
 */
namespace Config
{
    //Number of points from the start of previous_path_x/y that will be kept unchanged
    //so the new path is previous_path_x/y[0..kKeepPointsNum] + [new points].
    const unsigned long kKeepPointsNum              = 20;

    //Maximum allowed accelerations (m/s).
    const double        kMaxAcceleration            = 10;

    //Maximum allowed velocity (m/s).
    const double        kMaxVelocity                = Utils::MileHour2MeterSecond(50);

    //Preferred velocity
    const double        kTargetVelocity             = kMaxVelocity - Utils::MileHour2MeterSecond(5);

    //Time span between points consumption (s).
    const double        kTimeStep                   = 0.02;

    //Maximum velocity difference between two adjacent points so max acceleration is not violated.
    const double        kMaxVelocityDif             = kTimeStep * kMaxAcceleration;

    //Total number of points that will be passed back to the simulator.
    const unsigned long kHorizonPointsNum           = 70;

    //Distance between map way points.
    const double        kMapSegmentStep             = 15;

    //Number of points from the end of previous_path_x/y[0..kKeepPointsNum] that will be used for spline construction.
    const unsigned long kConnectPointsNum           = 2;

    //Width of the car that is used for collision detection.
    const double        kCarWidth                   = 2.2;

    //Length of the car that is used for collision detection.
    const double        kCarLength                  = 10;

    //Number of lanes.
    const unsigned long kLanesNum                   = 3;

    //Width of each lane.
    const double        kLaneWidth                  = 4;

    //Cost for zero velocity
    const double        kZeroVelocityCost           = 0.7;

    //Maximum velocity cost
    const double        kMaxVelocityCost            = 1.;

    //Number of times (calls from simulator) right after lane change, that next lane change will get high cost
    //E.g. we change lane, then for 50 times next lane change will have high cost and after 50 times the cost
    //will become regular
    const unsigned long kLaneChangeHighCostCount    = 50;

    //Cost for lane change when ego vehicle spent long enough time in current lane
    const double        kLaneChangeRegularCost      = 1;

    //Cost for lane change when ego vehicle did not spend long enough time in current lane
    const double        kLaneChangeHighCost         = 10000;

    //S-distance from the end of trajectory to the next car when next cost should have value at least kMaxVelocityCost
    //to override any velocity cost
    const double        kNextCostCriticalDistance   = 2;

    //Cars that are s-closer than this distance to the end of trajectory contribute to next cost
    const double        kNextCostMaxDistance        = 15;

    //Maximum cost for the car behind when it is at the position of the ego car
    const double        kPrevMaxCost                = 4;

    //When car behind is closer than this previous cost will be applied
    const double        kPrevMaxDistance            = 15;

    //First lane that is used when no lane was set before
    const int           kDefaultLane                = 1;

    //whether to print debug output
    const bool          kIsDebug                    = false;
};


#endif //PATH_PLANNING_CONSTANTS_H
