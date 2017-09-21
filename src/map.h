#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <vector>
#include <math.h>

/***
 * Contains map way points and helper methods
 */
class Map
{
public:
    //waypoints
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> s_;

    //maximum s value
    double max_s_;

public:
    unsigned long GetSize() const
    {
        return x_.size();
    }

    /***
     * Calculate s modulo max_s_
     * @param s
     * @return [0..max_s_]
     */
    double CorrectS(double s) const
    {
        return fmod(s, max_s_);
    }

    /***
     * Calculates distance between s values considering s values have loop.
     * In case the front car is more than half circumference away it is considered as being behind.
     * @param from_s
     * @param to_s
     * @return [-max_s/2..max_s/2]
     */
    double CalcSDistance(double from_s, double to_s)
    {
        from_s  = CorrectS(from_s);
        to_s    = CorrectS(to_s);

        double diff = to_s - from_s;
        double abs_diff = fabs(diff);
        if (abs_diff > max_s_ / 2.)
        {
            diff = (max_s_ - abs_diff) * (diff > 0 ? -1 : 1);
        }

        return diff;
    }
};


#endif //PATH_PLANNING_MAP_H
