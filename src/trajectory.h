#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <cassert>
#include <tuple>

#include "point.h"
#include "utils.h"
#include "config.h"

/***
 * Represents car trajectory. Contains list of points and target lane.
 * Target lane is needed because sometimes number of points in the trajectory may be not enough for the last point to
 * be on the target lane, so it may be impossible to understand what lane we are aiming at. At the same time for cost
 * calculation we need to know what target lane is.
 */
class Trajectory
{
private:
    //list of points. Each point is located kTimeStep seconds away from its adjacent point.
    std::vector<Point> points_;
    //e.g. for lane 1 it may be 0, 1, 2
    int target_lane_;

public:
    Trajectory() : target_lane_(Config::kDefaultLane) {}

    Trajectory(const Trajectory &trajectory, int target_lane)
    {
        points_         = trajectory.points_;
        target_lane_    = target_lane;
    }

    void AddPoint(Point point)
    {
        points_.push_back(point);
    }

    const Point& GetPoint(int idx) const
    {
        return points_[idx];
    };

    unsigned long GetSize() const
    {
        return points_.size();
    }

    Trajectory Subset(int start_idx, int points_num) const
    {
        Trajectory trajectory;

        for(int i = start_idx; i >= 0 && i < start_idx + points_num && i < GetSize(); i++)
        {
            trajectory.AddPoint(points_[i]);
        }

        return trajectory;
    }

    std::tuple<std::vector<double>, std::vector<double>> ToVectors()
    {
        std::vector<double> x;
        std::vector<double> y;

        for(auto &p : points_)
        {
            x.push_back(p.x_);
            y.push_back(p.y_);
        }

        return std::make_tuple(x, y);
    };

    int GetTargetLane()
    {
        return target_lane_;
    }

    int GetLastLane() const
    {
        double size = GetSize();
        return size > 0 ? Utils::ConvertD2LaneNum(GetPoint(size - 1).d_) : Config::kDefaultLane;
    }
};

#endif //PATH_PLANNING_TRAJECTORY_H
