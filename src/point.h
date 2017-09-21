#ifndef PATH_PLANNING_POINT_H
#define PATH_PLANNING_POINT_H

/***
 * Represents point.
 * Contains both cartesian and frenet coordinates.
 * Currently this is no necessary, but when I used brute force algorithms with a lot of possible trajectories,
 * converting coordinates on demand became expensive, so it was cheaper to do conversion one time.
 */
class Point
{
public:
    double x_;
    double y_;
    double s_;
    double d_;

    Point() : x_(0), y_(0), s_(0), d_(0) {}
    Point(double x, double y, double s, double d) : x_(x), y_(y), s_(s), d_(d) {}
};

#endif //PATH_PLANNING_POINT_H
