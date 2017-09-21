#ifndef PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define PATH_PLANNING_TRAJECTORY_GENERATOR_H

#include <vector>

#include "trajectory.h"
#include "map.h"
#include "spline.h"
#include "state.h"


/***
 * Helps to complete start trajectory using splines.
 */
struct ConnectionHelper
{
    //Contains trajectory that is used as start trajectory for spline creation
    //It might be last N points or current car location plus some extrapolated point
    //This used as the beginning of spline.
    Trajectory ref_trajectory_;

    //Velocity at the last point of ref_trajectory
    double     ref_velocity;
    
    //Angle at the last point of ref_trajectory
    double     ref_angle;

    const Point& GetLastPoint()
    {
        return ref_trajectory_.GetPoint(ref_trajectory_.GetSize() - 1);
    }
};

/***
 * Contains trajectory generation logic.
 */
class TrajectoryGenerator
{
public:

    /***
     * Creates list of trajectories for given lane and start conditions. The beginning of the result trajectory is
     * start_trajectory. The end of the trajectory consists of points with constant speed. Difference between last
     * start_trajectory speed and new speed is bounded by maximum acceleration.
     * Currently 3 trajectories are generated: with max acceleration, with max break, or same speed.
     *
     * @param start_trajectory
     * @param target_lane
     * @param cur_location      - ego location before start trajectory
     * @param cur_velocity      - ego velocity before start trajectory
     * @param points_num        - number of points needed to be generated to complete the trajectory
     * @param map
     * @return completed start trajectory
     */
    static std::vector<Trajectory> GetTrajectories(const Trajectory &start_trajectory, int target_lane, Point cur_location,
                                                   double cur_velocity, int points_num, const Map &map);
private:

    /***
     * Creates trajectory for given lane and start conditions. Spline is used to generate new points for the trajectory
     * to be smooth. I tried to use quintic polynomial, but it is hard to use it in the current setup because here
     * we need to find coordinates by trajectory length, I was unable to do this so the polynomial would be smooth
     * enough.
     *
     * Note that spline's frame is translated so its center is in the connection.ref_trajectory.GetLastPoint() and its
     * x axis is in the direction of connection.ref_angle. This method creates points using spline's frame and
     * translates them back to the original frame.
     *
     * @param start_trajectory  - previous points that we are going to reuse
     * @param target_lane       - target lane
     * @param connection        - contains translation point and rotation angle
     * @param spline            - spline of the trajectory in the translated frame see the main comment
     * @param velocity          - velocity using which the new trajectory should be generated
     * @param points_num        - number of points that should be generated
     * @param map               - map
     * @return trajectory in the given lane with given speed
     */
    static Trajectory GetTrajectory(const Trajectory &start_trajectory, int target_lane, ConnectionHelper &connection,
                                    tk::spline &spline, double velocity, int points_num, const Map &map);

    /***
     * Creates spline that is used to create smooth trajectory. Note that spline's frame is translated so its center
     * is in the last point of start trajectory and its x axis is aligned in the direction of the car when it is in
     * the last point of the start trajectory. The translation is needed for spline less likely create
     * vertical lines.
     *
     * @param connection - data that helps to create spline
     * @param lane       - target lane
     * @param map        - map
     * @return spline in translated and rotated frame
     */
    static tk::spline CalcSpline(ConnectionHelper &connection, int lane, const Map &map);


    static ConnectionHelper CreateConnectionHelper(const Trajectory &trajectory, const Point &cur_location,
                                                   double cur_velocity, const Map &map);


    /***
     * Not used at the moment.
     * @param s_0
     * @param s_1
     * @param s_2
     * @param e_0
     * @param e_1
     * @param e_2
     * @param t
     * @return
     */
    static std::vector<double> GetOptimalQuintic(double s_0, double s_1, double s_2,
                                                 double e_0, double e_1, double e_2,
                                                 double t);

    /***
     * Not used at the moment.
     * @param a
     * @param b
     * @param c
     * @return
     */
    std::tuple<double, double> GetQuadraticRoots(double a, double b, double c);
};


#endif //PATH_PLANNING_TRAJECTORY_GENERATOR_H
