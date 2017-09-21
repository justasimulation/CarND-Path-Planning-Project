#include <iostream>
#include <math.h>
#include <set>

#include "predictor.h"
#include "config.h"

using namespace std;

//currently only the original data from the simulator is used
//this method is redundant
CarPrediction Predictor::GetPrediction(int i, Map &map)
{
    CarPrediction prediction;

    auto o = observations_[i];

    auto frenet = Utils::Convert2Frenet(o.x_, o.y_, map);

    auto xy0 = Utils::ConvertToCartesian(frenet[0], 0, map);
    auto xy1 = Utils::ConvertToCartesian(frenet[0] + 1, 0, map);

    double dx = xy1[0] - xy0[0];
    double dy = xy1[1] - xy0[1];

    double v_square = o.vx_ * o.vx_ + o.vy_ * o.vy_;

    double cos_angle = (dx * o.vx_ + dy * o.vy_) / (sqrt(dx * dx + dy * dy) * sqrt(v_square));

    double v_s = sqrt(v_square);//sqrt(v_square) * cos_angle;
    double v_d = 0;//sqrt(v_square - v_s * v_s);

    for(int i = 0; i < Config::kHorizonPointsNum; i++)
    {
        double time = Config::kTimeStep * (i + 1);
        prediction.s_.push_back(frenet[0] + time * v_s);
        prediction.d_.push_back(frenet[1] + time * v_d);
    }

    return prediction;
}

uint Predictor::GetPredictionsNum()
{
    return observations_.size();
}

void Predictor::UpdatePredictions(std::vector<CarObservation> &sensor_fusion)
{
    observations_.clear();
    observations_ = sensor_fusion;
}

