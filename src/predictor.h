#ifndef PATH_PLANNING_PREDICTOR_H
#define PATH_PLANNING_PREDICTOR_H

#include <vector>
#include <map>
#include <deque>

#include "trajectory_generator.h"

/***
 * Observation received from the simulator.
 */
class CarObservation
{
public:
    int id_;
    double x_;
    double y_;
    double vx_;
    double vy_;
    double s_;
    double d_;
};

/***
 * Prediction of car locations over time.
 * Currently only the first location is used.
 */
class CarPrediction
{
public:
    std::vector<double> s_;
    std::vector<double> d_;
};

/***
 * Predicts other cars locations over time.
 * Currently this functionality is not used and original data from the simulator can be used instead.
 * But I kept the abstraction just in case.
 */
class Predictor
{
private:
    // observations from the simulator
    std::vector<CarObservation> observations_;

public:

    CarPrediction GetPrediction(int i, Map &map);
    uint GetPredictionsNum();
    void UpdatePredictions(std::vector<CarObservation> &sensor_fusion);
};


#endif //PATH_PLANNING_PREDICTOR_H
