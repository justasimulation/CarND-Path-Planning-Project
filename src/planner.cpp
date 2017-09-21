#include <iostream>
#include <iomanip>
#include <deque>
#include <stdlib.h>

#include "planner.h"
#include "config.h"
#include "cost.h"


Planner::Planner() : state_(State::kKeepLane), target_lane_(-1)
{
    map_ = Utils::LoadMap();
}

void Planner::UpdateTime()
{
    chrono::steady_clock::time_point cur_time_point = std::chrono::steady_clock::now();

    auto ms_num = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time_point - last_time_point_).count();

    if(Config::kIsDebug)
    {
        cout<<endl<<"Time: "<<ms_num<<endl;
    }

    last_time_point_ = cur_time_point;
}

/***
 *
 * @param input
 * @return
 */
Trajectory Planner::GetNextTrajectory(PlannerInput &input)
{
    if(Config::kIsDebug)
    {
        system("clear");
    }

    //1. Update time span between this method calls
    UpdateTime();

    //2. Calculate number of points to keep from the previous trajectory and number of points needed to generate again
    auto prev_path_size = input.prev_x_.size();

    int keep_points_num = std::min(prev_path_size, Config::kKeepPointsNum);
    int new_points_num = Config::kHorizonPointsNum - keep_points_num;

    //3. Create trajectory with needed number of points from the previous trajectory
    Trajectory start_trajectory;
    for(int i = 0; i < keep_points_num && i < prev_path_size; i++)
    {
        auto point = Utils::CreatePointByCartesian(input.prev_x_[i], input.prev_y_[i], map_);
        start_trajectory.AddPoint(point);
    }

    //4. Calculate the best trajectory
    auto cur_location = Utils::CreatePointByCartesian(input.x_, input.y_, map_);

    return GetBestTrajectory(start_trajectory, cur_location, input.velocity_, new_points_num, input.sensor_fusion_);
}


/***
 *
 * @param start_trajectory
 * @param cur_point
 * @param cur_velocity
 * @param new_points_num
 * @param sensor_fusion
 * @return
 */
Trajectory Planner::GetBestTrajectory(Trajectory &start_trajectory, Point cur_location, double cur_velocity,
                                      int new_points_num, std::vector<CarObservation> sensor_fusion)
{
    //1. Set predictions in case we use real sensor fusion (currently we don't)
    predictor_.UpdatePredictions(sensor_fusion);

    //2. Set minimum cost to high value so it will be overriden by any cost
    double min_cost = std::numeric_limits<double>::max();

    //3. Declare new values that will be calculated by this method
    State       new_state = state_;
    Trajectory  new_trajectory;
    int         new_target_lane;
    int         new_index;

    //4. Get list of possible states
    auto states = GetPossibleStates(start_trajectory.GetLastLane());

    //5. Iterate over states to calculates best trajectory
    for(auto &state : states)
    {
        //6. Calculate target lane for current state
        int target_lane = start_trajectory.GetLastLane() + (state == State::kChangeLaneRight ? 1 :
                                                            (state == State::kChangeLaneLeft ? -1 : 0));

        //7. Calculate best trajectory for the state and its cost
        double cost;
        int index;
        Trajectory trajectory;

        tie(cost, index, trajectory) = GetBestTrajectoryForLane(target_lane, start_trajectory,
                                                                cur_location, cur_velocity, new_points_num);

        //8. In case the cost got better keep the trajectory
        if(cost < min_cost)
        {
            min_cost = cost;

            new_state = state;
            new_trajectory = trajectory;
            new_target_lane = target_lane;
            new_index = index;
        }
    }

    //9. Set newly calculated cost and target lane
    target_count_ = target_lane_ == new_target_lane ? target_count_ + 1 : 0;
    target_lane_  = new_target_lane;
    state_        = new_state;

    if(Config::kIsDebug)
    {
        vector<string> actions = {"break", "no", "speed"};
        vector<string> states = {"keep", "left", "right"};

        cout<<"State: "<<states[state_]<<endl;
        cout<<"Action: "<<actions[new_index]<<endl;
        cout<<"Lane: "<<target_lane_<<endl;
    }

    //10. Return the best trajectory
    return new_trajectory;
}

tuple<double, int, Trajectory> Planner::GetBestTrajectoryForLane(int target_lane,
                                                            Trajectory &start_trajectory,
                                                            Point cur_location, double cur_velocity,
                                                            int new_points_num)
{
    //get possible trajectories
    auto trajectories = TrajectoryGenerator::GetTrajectories(start_trajectory, target_lane, cur_location,
                                                             cur_velocity, new_points_num, map_);

    //find first trajectory cost
    int     min_trajectory_idx = 0;
    double  min_cost = Cost::CalcCost(trajectories[0], target_lane_, target_count_, predictor_, map_);

    //find minimum trajectory cost
    for(int i = 1; i < trajectories.size(); i++)
    {
        double cost = Cost::CalcCost(trajectories[i], target_lane_, target_count_, predictor_, map_);
        if(cost < min_cost)
        {
            min_cost            = cost;
            min_trajectory_idx  = i;
        }
    }

    if(Config::kIsDebug)
    {
        cout<<endl;
    }

    return make_tuple(min_cost, min_trajectory_idx, trajectories[min_trajectory_idx]);

}


vector<State> Planner::GetPossibleStates(int lane)
{
    //keep state is allowed for all lanes.

    //for the left most lane (0) only right change is allowed,
    //for the right most lane only left change is allowed,
    //otherwise both changes are allowed.
    vector<State> states;

    states.push_back(State::kKeepLane);

    if(Config::kLanesNum > 1)
    {
        switch(lane)
        {
            case 0:
                states.push_back(State::kChangeLaneRight);
                break;
            case Config::kLanesNum - 1:
                states.push_back(State::kChangeLaneLeft);
                break;
            default:
                states.push_back(State::kChangeLaneLeft);
                states.push_back(State::kChangeLaneRight);
        }
    }

    return states;
}

