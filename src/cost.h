#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

#include "trajectory.h"
#include "predictor.h"


/***
 *Trajectory cost is calculated here.
 */
class Cost
{
public:
    /***
     * Calculates given trajectory cost. Cost should be lower for better trajectories.
     *
     * Cost contains of:
     * 1. Velocity cost
     * 2. Distance to the next car in the target lane cost
     * 3. Distance to the previous car in the target lane cost
     * 4. Change lane cost
     *
     * Acceleration cost is not considered because in practice it is almost always correct, so it is hard to debug it.
     *
     * In practice, considering design of Planner class:
     * -velocity doesn't affect lane change much and only affects speed
     * -next distance cost in the current lane forces car to change lane
     * -next distance, previous distance and lane change cost in other lanes force car to stay in the same lane
     *
     * In case we are considering lane change under pressure of next distance cost:
     * 1. In case another lane has only next distance cost, then we'll choose the best distance by comparing costs
     * 2. In case another lane has only previous distance cost, that means that previous distance cost in critical areas
     * of possible collision should have values as high as next distance cost to overcome its pressure.
     * Currently this item is not implemented ideally. Previous cost values are manually tweaked, but so far it was
     * enough.
     * 3. In case we have both next and previous cost in adjacent lane and in case item 2 is implemented correctly
     * we'll choose the correct lane by comparing cost.
     *
     * Lane change cost has two values: large and small. Large value is applied right after lane change to prevent
     * fast random lane changes. Those random lane changes were observed in case when a car in front of the ego car
     * did lane change at the same time as ego car putting itself right in front of the ego car or to another lane.
     * The cost is large enough to override all other costs, this gives time for things to calm down. After some time
     * the cost becomes very small and only prevents random lane change when velocity cost is almost identical for
     * adjacent lanes.
     *
     *
     * @param trajectory        - trajectory
     * @param prev_target_lane  - last target lane
     * @param prev_target_count - number of times (calls from simulator) that last target lane was unchanged
     * @param predictor         - a class to get other cars predictions from
     * @param map               - map
     * @return non negative value
     */
    static double CalcCost(Trajectory &trajectory,
                           int prev_target_lane, int prev_target_count,
                           Predictor &predictor, Map &map);

private:

    /***
     * Calculates velocity cost. Cost tend to be lower for target velocity. This helps keeping the optimal speed.
     *
     * Velocities below 0 and above kMaxVelocity cost kMaxCostVelocity, otherwise it is linear from
     * kZeroVelocityCost at 0, then 0 at kTargetVelocity, then quadratic to kVelocityMaxCost at kMaxVelocity.
     *
     * @param trajectory
     * @return [0..kVelocityMaxCost]
     */
    static double CalcVelocityCost(Trajectory &trajectory);

    /***
     * Calculates lane change cost. Cost tend to be lower for staying at the same lane and particularly
     * staying at the same lane right after lane change.
     *
     * Sometimes costs of different lane trajectories are almost identical which may cause random lane change, so
     * this method adds some cost for lane change.
     * Sometimes right after lane  when configuration changes significantly this small cost is not enough
     * which causes fast random lane change, so this method adds huge cost for lane change for some time right after
     * lane change (e.g. 1000 for 1 second). This prevents fast random lane changes which could cause sharp curves.
     *
     * @param trajectory        - trajectory
     * @param prev_target_lane  - previous target lane
     * @param prev_target_count - number of times (calls from simulator) that last target lane was unchanged
     * @return [0, kLaneChangeRegularCost, kLaneChangeHighCost]
     */
    static double CalcLaneChangeCost(Trajectory &trajectory, int prev_target_lane, int prev_target_count);

    /***
     * Calculates cost for distance to the closest car that is in the target lane and is in front of the ego car. Cost
     * is calculated relative to the end of the trajectory. Cost tends to be lower for more distant closest cars.
     *
     * When a car is closer than kNextCostMaxDistance to the end of trajectory it linearly increases from 0 to
     * kVelocityMaxCost when it is at kNextCostCritical distance and increases further. It means that when the car is
     * kNextCostCriticalDistance from the end of trajectory it overrides any velocity cost.
     *
     * @param trajectory    - trajectory
     * @param predictor     - a class to get other cars predictions from
     * @param map           - map
     * @return [0.. (s_start - s_end) * kVelocityMaxCost/(kNextCostCriticalDistance - kNextCostMaxDistance) -
     *              kNextCostMaxDistance * kVelocityMaxCost/(kNextCostCritical - kNextCostMax)]
     */
    static double CalcNextDistanceCost(Trajectory &trajectory, Predictor &predictor, Map &map);


    /***
     * Calculates cost for distance to the closest car that is located in the target lane and is behind the ego car.
     * When target lane is the same as the ego car's one than cost is 0, because this cost is needed only for lane change
     * evaluation.
     *
     * When a car behind is closer than kPrevCostTimeHorizon seconds of maximum velocity away from the ego car, the cost
     * increases from 0 to kPrevMaxCost at the ego car's location.
     *
     * @param trajectory    - trajectory
     * @param predictor     - a class to get other cars predictions from
     * @param map           - map
     * @return [0..Config::kPrevMaxCost]
     */
    static double CalcPrevDistanceCost(Trajectory &trajectory, Predictor &predictor, Map &map);
};


#endif //PATH_PLANNING_COST_H
