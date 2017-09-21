#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include <chrono>

#include "predictor.h"
#include "trajectory.h"
#include "state.h"

/***
 * Convenience class to pass parameters from simulator to planner
 */
class PlannerInput
{
public:
    double x_;
    double y_;
    double velocity_;
    std::vector<double> prev_x_;
    std::vector<double> prev_y_;
    std::vector<CarObservation> sensor_fusion_;
};

/***
 * The main class. Server gets data from simulator then passes it here then recieves the result
 * and returns it to the simulator.
 */
class Planner
{
private:

    //time to check how expensive calculation is
    std::chrono::steady_clock::time_point last_time_point_ = std::chrono::steady_clock::now();

    //ego car state
    State state_;

    //last target lane
    int target_lane_;
    //how may times last target lane stayed unchanged
    int target_count_;

    //map waypoints
    Map map_;

    //wrapper class for sensor fusion data
    Predictor predictor_;

public:
    Planner();

    /***
     * Returns the best trajectory for given input. kKeepPointsNum points from the previous points received from the
     * simulator are reused and new points are added so the overall number is kHorizonPointsNum.
     *
     * @param input
     * @return trajectory with the lowest cost
     */
    Trajectory GetNextTrajectory(PlannerInput &input);

private:

    /***
     * Updates time stamp.
     */
    void UpdateTime();

    /***
     * Calculates rest of trajectory for given start trajectory given current point, velocity and other cars locations.
     *
     * @param start_trajectory  - previous points received from simulator that we are going to reuse
     * @param cur_point         - current ego car location before start_trajectory
     * @param cur_velocity      - current velocity before start_trajectory
     * @param new_points_num    - number of points needed to create to complete the trajectory
     * @param sensor_fusion     - other cars observations
     * @return trajectory consisting of kHorizonPointsNum points with lowest cost
     */
    Trajectory GetBestTrajectory(Trajectory &start_trajectory, Point cur_point, double cur_velocity,
                                 int new_points_num, std::vector<CarObservation> sensor_fusion);

    /***
     * Calculates rest of trajectories for given start trajectory for given lane, chooses the one with the lowest cost
     * and returns it alongside with the cost.
     *
     * @param target_lane       - lane that we're going to change to
     * @param start_trajectory  - kKeepPointsNUm of previous points from the simulator
     * @param cur_point         - current ego car location before start_trajectory
     * @param cur_velocity      - current ego car velocity before start_trajectory
     * @param new_points_num    - number of points needed to be generated, the overall number should be kHorizonPointsNum
     * @return <cost, trajectory index, lowest cost trajectory consisting of kHorizonPointsNum>
     */
    std::tuple<double, int, Trajectory> GetBestTrajectoryForLane(int target_lane,
                                                                 Trajectory &start_trajectory,
                                                                 Point cur_point, double cur_velocity,
                                                                 int new_points_num);


    /***
     * Returns possible states for given lane.
     * @param lane - lane number
     * @return [keep, left, right]
     */
    std::vector<State> GetPossibleStates(int lane);
};


#endif //PATH_PLANNING_PLANNER_H
