#include <iostream>
#include <iomanip>
#include <math.h>

#include "cost.h"

#include "config.h"

using namespace std;


double Cost::CalcCost(Trajectory &trajectory,
                      int prev_target_lane, int prev_target_count,
                      Predictor &predictor, Map &map)
{
    //names for debug output
    vector<string> names            = {"velocity", "forward", "backward", "lane_change"};
    //weights for each type of cost
    vector<double> weights          = {1, 1, 1, 0.005};

    vector<double> costs            = {0, 0, 0, 0};
    vector<double> weighted_costs   = {0, 0, 0, 0};

    //calculate costs
    costs[0] = CalcVelocityCost(trajectory);
    costs[1] = CalcNextDistanceCost(trajectory, predictor, map);
    costs[2] = CalcPrevDistanceCost(trajectory, predictor, map);
    costs[3] = CalcLaneChangeCost(trajectory, prev_target_lane, prev_target_count);

    //weight the costs
    for (int i = 0; i < costs.size(); i++)
    {
        weighted_costs[i] = costs[i] * weights[i];
    }

    //calculate overall cost
    double cost = 0;
    for (auto &c : weighted_costs)
    {
        cost += c;
    }

    if(Config::kIsDebug)
    {
        cout<<fixed<<setprecision(4);
        for (int i = 0; i < weighted_costs.size(); i++)
        {
            cout<<names[i]<<": "<<weighted_costs[i]<<" ";
        }
        cout<<endl;
    }

    return cost;
}

double Cost::CalcVelocityCost(Trajectory &trajectory)
{
    double cost = 0;

    for (int i = 1; i < trajectory.GetSize(); i++)
    {
        //calculate distance between two consecutive points
        auto cur_point = trajectory.GetPoint(i);
        auto prev_point = trajectory.GetPoint(i - 1);

        double dist = Utils::CalcDistance(prev_point, cur_point);

        //calculate instant velocity at point i
        double velocity = dist / Config::kTimeStep;

        //coefficients for linear decay in [0, kTargetVelocity] interval: a * v + b
        double b = Config::kZeroVelocityCost;
        double a = -b / Config::kTargetVelocity;

        //coefficients for quadratic ascend in [kTargetVelocity, kMaxVelocity]: c * (v - d)^2 + e
        double d = Config::kMaxVelocity;
        double e = Config::kMaxVelocityCost;
        double c = -e / pow(Config::kMaxVelocity - Config::kTargetVelocity, 2);

        double local_cost = velocity < 0 ? Config::kMaxVelocityCost : // (-inf, 0) -> max
                            (velocity < Config::kTargetVelocity ?     // [0, target] -> a * v + b
                             (a * velocity + b) :
                             (velocity < Config::kMaxVelocity ?       // [target, max] -> c * (v - d)^2 + e
                              c * pow(d - velocity, 2) + e :
                              Config::kMaxVelocityCost));             // [max, inf) -> max

        cost += local_cost;
    }

    cost = cost / trajectory.GetSize(); // make it average so number of points doesn't affect the cost

    return cost;
}

double Cost::CalcLaneChangeCost(Trajectory &trajectory, int prev_target_lane, int prev_target_count)
{
    return trajectory.GetTargetLane() == prev_target_lane ? 0 :             // No cost for no change lane.
           (prev_target_count > Config::kLaneChangeHighCostCount ?          // Regular cost if we are long enough
            Config::kLaneChangeRegularCost : Config::kLaneChangeHighCost);  // in this lane, otherwise high cost.
}


double Cost::CalcNextDistanceCost(Trajectory &trajectory, Predictor &predictor, Map &map)
{
    double target_lane_center_d = Utils::GetLaneCenterD(trajectory.GetTargetLane());
    double ego_start_s          = trajectory.GetPoint(0).s_;
    double ego_end_s            = trajectory.GetPoint(trajectory.GetSize() - 1).s_;

    //calc linear coefficients so the cost is 0 at max distance and kMaxVelocityCost at critical distance
    //being kMaxVelocityCost at critical distance means that this cost will override any velocity cost when a car is closer than that
    double a = Config::kMaxVelocityCost / (Config::kNextCostCriticalDistance - Config::kNextCostMaxDistance);
    double b = -Config::kNextCostMaxDistance * a;

    double max_cost = 0;

    //iterate over predictions
    for(uint i = 0; i < predictor.GetPredictionsNum(); i++ )
    {
        //get prediction
        auto car = predictor.GetPrediction(i, map);

        //consider only cars that are in the target lane
        if(fabs(car.d_[0] - target_lane_center_d) < ((Config::kCarWidth / 2) + (Config::kLaneWidth / 2)))
        {
            //calc s distance to the target car from the ego car
            double dist = map.CalcSDistance(ego_start_s, car.s_[0]);
            if(dist >= 0) // consider only cars that are in front of the ego vehicle
            {
                //calc s distance from the end of the trajectory to the target car
                dist = map.CalcSDistance(ego_end_s, car.s_[0]);
                if(dist <= Config::kNextCostMaxDistance)
                {
                    double cost = a * dist + b;
                    max_cost = cost > max_cost ? cost : max_cost;
                }
            }
        }
    }

    return max_cost;
}


double Cost::CalcPrevDistanceCost(Trajectory &trajectory, Predictor &predictor, Map &map)
{
    double max_cost = 0;

    auto start_point = trajectory.GetPoint(0);

    //Do not calculate the price if target lane is the same lane that we are now
    //We want to have this cost only for changing lanes scenario
    if(Utils::ConvertD2LaneNum(start_point.d_) != trajectory.GetTargetLane())
    {
        double target_lane_center_d = Utils::GetLaneCenterD(trajectory.GetTargetLane());

        //calc coefficients so cost linearly increases from 0 at KPrevCostTimeHorizon seconds of max velocity behind the ego car
        //and reaches kPrevMaxCost at the ego car location
        double b =  Config::kPrevMaxCost;
        double a = -b / Config::kPrevMaxDist;

        //check every prediction
        for(uint i = 0; i < predictor.GetPredictionsNum(); i++ )
        {
            auto car = predictor.GetPrediction(i, map);

            //check cars that are in the target lane
            if(fabs(car.d_[0] - target_lane_center_d) < ((Config::kCarWidth / 2) +  (Config::kLaneWidth / 2)))
            {
                double dist = -map.CalcSDistance(start_point.s_, car.s_[0]);
                if(dist >= 0 && dist <= Config::kPrevMaxDist) //consider only cars that are behind the ego and closer than kPrevTimeHorizon seconds away
                {
                    double cost = a * dist + b;
                    max_cost = cost > max_cost ? cost : max_cost;
                }
            }
        }
    }

    return max_cost;

}
