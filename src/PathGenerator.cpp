//
// Created by Jose Rojas on 7/23/17.
//

#include "PathGenerator.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Geometry"
#include "SplineLibrary/spline_library/splines/natural_spline.h"

typedef Vector<2, double> Vector2d;
typedef NaturalSpline<Vector2d,double> SplineType;


typedef struct {
    int id;
    int lane;
    double start;
    double end;
    double speed;
} CollisionBoundingBox;

typedef struct {
    std::vector<double> JMTparams;
    double cost;
} JMTCurve;

typedef struct {
    std::vector<double> sJMTparams;
    std::vector<double> dJMTparams;
    double dTimeToNextJMT;
    double dTimeOffsetJMT;
    double sTimeToNextJMT;
    double sTimeOffsetJMT;
    double currentLane;
    double dt;
} TrajectoryData;

typedef std::vector<CollisionBoundingBox> VecCollisionBoundingBox;

static Eigen::Rotation2D<double> g_rotNormal(-90.f * M_PI / 180.f);

extern double distance(double x1, double y1, double x2, double y2);

double JMTeval(std::vector<double> params, double T) {
    double t2 = T * T;
    double t3 = t2 * T;
    double t4 = t3 * T;
    double t5 = t4 * T;
    return params[0] + params[1] * T + params[2] * t2 + params[3] * t3 + params[4] * t4 + params[5] * t5;
}

double JMTSpeedEval(std::vector<double> params, double T) {
    double t2 = T;
    double t3 = t2 * T;
    double t4 = t3 * T;
    double t5 = t4 * T;
    return params[1] + 2 * params[2] * t2 + 3 * params[3] * t3 + 4 * params[4] * t4 + 5 * params[5] * t5;
}

double JMTAccelEval(std::vector<double> params, double T) {
    double t3 = T;
    double t4 = t3 * T;
    double t5 = t4 * T;
    return 2 * params[2] + 6 * params[3] * t3 + 12 * params[4] * t4 + 20 * params[5] * t5;
}

double JMTJerkEval(std::vector<double> params, double T) {
    double t4 = T;
    double t5 = t4 * T;
    return 6 * params[3] + 24 * params[4] * t4 + 60 * params[5] * t5;
}

Eigen::Vector2d getNormalFromCurvature(SplineType::InterpolatedPTC& curvature, double t) {
    Vector2d v = curvature.position;
    Vector2d tx = curvature.tangent;
    double x = v[0];
    double y = v[1];

    //rotate tangent by -90 degrees to produce normal
    Eigen::Vector2d tv;
    tv << tx[0], tx[1];
    Eigen::Vector2d normal = g_rotNormal * tv;
    normal.normalize();

    return normal;
}

double getCurvatureScalerFromCurvature(SplineType::InterpolatedPTC& curvature, double t) {
    Vector2d cx = curvature.curvature;
    Vector2d v = curvature.position;
    Vector2d tx = curvature.tangent;
    double x = v[0];
    double y = v[1];
    double x_dot = tx[0];
    double y_dot = tx[1];
    double x_ddot = cx[0];
    double y_ddot = cx[1];

    return (x_dot * y_ddot - y_dot * x_ddot) / pow(x_dot * x_dot + y_dot * y_dot, 1.5);
}

std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    Eigen::MatrixXd A(3, 3);
    Eigen::VectorXd b(3);

    double t2 = pow(T, 2);
    double t3 = pow(T, 3);
    double t4 = pow(T, 4);
    double t5 = pow(T, 5);

    A << t3, t4, t5,
            3 * t2, 4 * t3, 5 * t4,
            6 * T, 12 * t2, 20 * t3;

    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * t2),
            end[1] - (start[1] + start[2] * T),
            end[2] - start[2];

    Eigen::MatrixXd Ainv = A.inverse();

    Eigen::VectorXd x = Ainv * b;

    return {start[0],start[1], 0.5 * start[2], x(0), x(1), x(2)};

}

bool validateJMT(std::vector<double> params, double dT, double dt, double max_velocity, double max_accel, double max_jerk) {
    //does it violate acceleration
    bool failedConstraint = false;
    for (double t = 0; t < dT && !failedConstraint; t += dt) {
        if (fabs(JMTAccelEval(params, t)) > max_accel) {
            failedConstraint = true;
            break;
        }
    }

    //does it violate speed
    for (double t = 0; t < dT && !failedConstraint; t += dt) {
        if (fabs(JMTSpeedEval(params, t)) > max_velocity) {
            failedConstraint = true;
            break;
        }
    }

    //does it violate jerk
    for (double t = 0; t < dT && !failedConstraint; t += dt) {
        if (fabs(JMTJerkEval(params, t)) > max_jerk) {
            failedConstraint = true;
            break;
        }
    }

    return !failedConstraint;
}

JMTCurve permuteJMT(double pos, double speed, double accel, double final_s, double final_velocity, double max_velocity, double max_accel, double max_jerk, double dT, double dt, bool bPermuteS = true) {
    JMTCurve bestCurve;
    bestCurve.cost = 0;

    double max_s = (pos < final_s ? max_velocity : -max_velocity)  * dT + pos;
    double s_ddot = 0.0;
    double s_ddot_inc = speed > final_velocity ? -0.05 : 0.05;
    double s_ddot_final = speed > final_velocity ? -max_accel : max_accel;

    if (fabs(speed - max_velocity) < 0.01) {
        speed = max_velocity;
    }

    while (((final_velocity >= speed && s_ddot <= s_ddot_final) || (final_velocity <= speed && s_ddot >= s_ddot_final)) &&
            fabs(s_ddot) <= fabs(s_ddot_final)) {

        double s_dot = final_velocity;
        double s_dot_inc = speed < final_velocity ? -0.5 : 0.5;

        while (((final_velocity >= speed && s_dot >= speed) || (final_velocity <= speed && s_dot <= speed)) &&
                fabs(s_dot) <= fabs(max_velocity)) {
            double s = final_s;
            int factor = 0;

            auto diff = (int) (fabs(final_s - pos) / 0.5f);
            if (!bPermuteS)
                diff = 0;

            while (factor <= diff) {

                double sign = (factor % 2) == 0 ? -1.f : 1.f;

                s = final_s + factor * sign * 0.5;
                std::vector<double> start{pos, speed, accel};
                std::vector<double> end{s, s_dot, s_ddot};

                std::vector<double> params = JMT(start, end, dT);

                if (validateJMT(params, dT, dt, max_velocity, max_accel, max_jerk)) {
                    bestCurve.JMTparams = params;
                    printf("JMT final (pos %f, speed %f, accel %f)\n", s, s_dot, s_ddot);
                    return bestCurve;
                }

                factor++;
            }

            s_dot += s_dot_inc;
        }

        s_ddot += s_ddot_inc;
    }

    return bestCurve;
}

std::vector<CollisionBoundingBox> calculateLaneBoundingBoxForSensorVehicleState(SensorVehicleState state, double dt, double laneWidth) {
    //XXX assume cars do not switch lanes for now

    double direction = 1.f; //state.yaw > M_PI/4 ? 1.f : -1.f;
    double s1 = state.s;
    double s2 = state.s + (state.speed * dt) * direction;
    int lane = (int) (state.d / laneWidth);

    return {{state.id,lane,s1,s2,state.speed}};
}

std::vector<CollisionBoundingBox> calculateLaneBoundingBox(double d, double s, double speed, double dt, double laneWidth) {
    double s1 = s;
    double s2 = s + (speed * dt);
    int lane = (int) (d / laneWidth);
    return {{0,lane,s1,s2,speed}};
}

double calculateCollisionMoment(SensorVehicleState state, const std::vector<double>& jmtParams, double d, double dT, double dt, double laneWidth, double range) {
    double t = 0;
    double start = JMTeval(jmtParams, 0);
    double s = 0;
    while (t <= dT) {
        s = JMTeval(jmtParams, t);
        double vs = state.s + t * state.speed;
        bool right = (state.d + (laneWidth / 2) >= d - (laneWidth / 2)) && (state.d + (laneWidth / 2) <= d + (laneWidth / 2));
        bool left = (state.d - (laneWidth / 2) >= d - (laneWidth / 2)) && (state.d - (laneWidth / 2) <= d + (laneWidth / 2));
        if (s >= vs && (right || left)) {
            return s;
        }
        t += dt;
    }
    return -1;
}

bool does_overlap(std::vector<CollisionBoundingBox>& boxes1, std::vector<CollisionBoundingBox>& boxes2) {

    bool overlap = false;

    for (CollisionBoundingBox& box1 : boxes1) {
        for (CollisionBoundingBox& box2 : boxes2) {
            overlap |= ((box2.start >= box1.start && box2.start <= box1.end) || (box2.end >= box1.start && box2.end <= box1.end))
                       && box1.lane == box2.lane
                       && box1.start <= box2.start;
        }
    }

    return overlap;
}

bool does_collide(std::vector<CollisionBoundingBox>& boxes, int id) {
    for (CollisionBoundingBox& box : boxes) {
        if (box.id == id)
            return true;
    }
    return false;
}

std::chrono::milliseconds currentTime() {
    return std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );
}

double arcLength(const std::vector<Vector2d>& points, int start, int end) {
    double length = 0;
    if (points.size() > start) {
        Vector2d lastPoint = points[start];
        for (int i = start + 1; i < end; i++) {
            const Vector2d& newPoint = points[i];
            length += distance(lastPoint[0], lastPoint[1], newPoint[0], newPoint[1]);
            lastPoint = newPoint;
        }
    }
    return length;
}

std::vector<Vector2d> xyVectorsToVector2d(const std::vector<double>& xs, const std::vector<double>& ys) {
    std::vector<Vector2d> retval;
    for (int i = 0; i < xs.size(); i++) {
        Vector2d x;
        x[0] =xs[i];
        x[1] =ys[i];
        retval.push_back(x);
    }
    return retval;
}

Waypoints generate_scaled_waypoints_from_points(std::vector<Vector2d> xv, double s, double s_max, double sourceInterval, double destInterval) {

    SplineType spline(xv);

    // the original waypoints are not uniformly distributed and our cubic hermite spline requires uniformly distributed
    // points - regenerate a set of waypoints with a uniform 's' distribution

    Waypoints newWaypoints;

    double t = s, err = 0;
    xv.clear();

    while (s < s_max) {

        Vector2d pos = spline.getPosition(t / sourceInterval);
        SplineType::InterpolatedPTC tangent = spline.getCurvature(t / sourceInterval);
        Eigen::Vector2d normal = getNormalFromCurvature(tangent, t / sourceInterval);

        Vector2d v = tangent.position;
        double x = v[0];
        double y = v[1];
        newWaypoints.x.push_back(x);
        newWaypoints.y.push_back(y);
        newWaypoints.s.push_back(s);
        newWaypoints.dx.push_back(normal[0]);
        newWaypoints.dy.push_back(normal[1]);

        xv.push_back(v);
        Vector<1, double> tx;
        tx[0] = (float) t;

        double length = 0;
        double dt = 1.0, factor = 1.f;
        bool incu = false, incd = false;
        double precision = 0.00001;
        do {
            //generate a new set of waypoints that are uniformly separated by arc-length
            Vector2d x1 = spline.getPosition(t / sourceInterval);
            Vector2d x2 = spline.getPosition((t + dt) / sourceInterval);
            length = distance(x1[0], x1[1], x2[0], x2[1]);
            if (length - destInterval > precision) {
                dt -= factor;
                incu = true;
            } else if (length - destInterval < -precision) {
                dt += factor;
                incd = true;
            }

            if (incu && incd) {
                if (factor < precision * 0.01) {
                    precision /= 0.1;
                    factor /= 100;
                } else {
                    factor *= 0.1;
                    incu = incd = false;
                }
            }

        } while (fabs(length - destInterval) > precision);

        s += destInterval;
        t += dt;
        err += (length - destInterval);
    }

    printf("Total error = %f\n", err);

    return newWaypoints;
}


class PathGenerator::impl {
public:

    impl();
    ~impl();
    int find_waypoint_floor(std::vector<double>& waypoints, double s);
    double find_target_t(std::vector<double>& waypoints, double s);
    double find_closest_time(double s);
    TrajectoryData generate_constraint_JMT(VehicleState state, double pos, double max_velocity, double max_accel, double max_jerk);
    VecCollisionBoundingBox check_overlap(double d, const std::vector<SensorVehicleState>& sensor_state, const std::vector<double>& params, double dT, double dt);
    VecCollisionBoundingBox check_collisions(const std::vector<SensorVehicleState>& vehicles,
                                             const VecCollisionBoundingBox& boxes,
                                             double d,
                                             const std::vector<double>& params,
                                             double dT,
                                             double dt);
    std::vector<Vector2d> generate_path_points(SplineType* spline, Waypoints& waypoints,
                                               const TrajectoryData& jmtParams, std::vector<Vector2d> points,
                                               int startI, double s_start, double s_end, double s_max,
                                               int toKeep, int maxItems, bool validate);

    Waypoints waypoints_;
    double s_max_;
    SplineType* spline_;
    TrajectoryData trajectory;
    double dT;
    double dt;
    double maxToKeep;
    double uniformInterval;
    double minimumFollowDistance;
    std::vector<Vector2d> prevPoints;
    CollisionBoundingBox collisionState;

    std::vector<double> getFrenet(VehicleState state, double x, double y, std::vector<double> maps_x, std::vector<double> maps_y) {

        int i = 0, next_wp, prev_wp;
        double dist = waypoints_.s.size() > 0 ? waypoints_.s[waypoints_.s.size() - 1] : 0, prevDistance = 0;
        while (i < waypoints_.s.size()) {
            double nextDistance = distance(waypoints_.x[i], waypoints_.y[i], x, y);
            if (nextDistance > dist) {
                if (prevDistance < nextDistance) {
                    next_wp = i - 1;
                    prev_wp = i - 2;
                } else {
                    next_wp = i;
                    prev_wp = i - 1;
                }
                break;
            }
            prevDistance = dist;
            dist = nextDistance;
            i++;
        }

        double n_x = maps_x[next_wp]-maps_x[prev_wp];
        double n_y = maps_y[next_wp]-maps_y[prev_wp];
        double x_x = x - maps_x[prev_wp];
        double x_y = y - maps_y[prev_wp];

        // find the projection of x onto n
        double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        double proj_x = proj_norm*n_x;
        double proj_y = proj_norm*n_y;

        double frenet_d = distance(x_x,x_y,proj_x,proj_y);

        //see if d value is positive or negative by comparing it to a center point

        double center_x = 1000-maps_x[prev_wp];
        double center_y = 2000-maps_y[prev_wp];
        double centerToPos = distance(center_x,center_y,x_x,x_y);
        double centerToRef = distance(center_x,center_y,proj_x,proj_y);

        if(centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }

        // calculate s value
        double frenet_s = waypoints_.s[prev_wp];

        frenet_s += distance(0,0,proj_x,proj_y);

        return {frenet_s,frenet_d};

    }

};

PathGenerator::PathGenerator() : pimpl(new impl()) {

}

PathGenerator::~PathGenerator() = default;

PathGenerator::PathGenerator(Waypoints waypoints, double s_max) : PathGenerator() {

    impl& im = *pimpl;
    im.s_max_ = s_max;
    im.maxToKeep = 10;
    im.uniformInterval = 1.f;
    im.minimumFollowDistance = 20.f;

    std::vector<Vector2d> xv = xyVectorsToVector2d(waypoints.x, waypoints.y);
    Waypoints newWaypoints = generate_scaled_waypoints_from_points(xv, 0, s_max, im.uniformInterval, im.uniformInterval);
    xv = xyVectorsToVector2d(newWaypoints.x, newWaypoints.y);

    im.spline_ = new SplineType(xv);
    im.waypoints_ = newWaypoints;
    im.dT = 4.f;
    im.dt = 0.02;
    im.trajectory.dt = im.dt;
}


PathPoints PathGenerator::generate_path(VehicleState state) {

    impl& im = *pimpl;

    int used = (int) fmax(0, (int) im.prevPoints.size() - (int) state.remaining_path_x.size());
    unsigned long toKeep = (unsigned long) fmin(im.maxToKeep, state.remaining_path_x.size());
    double lengthDistTraveled = arcLength(im.prevPoints, 0, used);

    //reduce time
    im.trajectory.sTimeToNextJMT -= used * im.trajectory.dt;
    im.trajectory.sTimeOffsetJMT += used * im.trajectory.dt;
    im.trajectory.dTimeToNextJMT -= used * im.trajectory.dt;
    im.trajectory.dTimeOffsetJMT += used * im.trajectory.dt;

    printf("used %d, time delta %f, distance traveled %f, speed %f, accel %f\n",
           used, used * im.trajectory.dt, lengthDistTraveled,
           JMTSpeedEval(im.trajectory.sJMTparams, im.trajectory.sTimeOffsetJMT),
           JMTAccelEval(im.trajectory.sJMTparams, im.trajectory.sTimeOffsetJMT)
    );

    bool bSkip = im.trajectory.sTimeToNextJMT > 0.0001;

    // add previous points
    PathPoints retval;
    std::vector<Vector2d> points;
    int i = 0;
    for (; i < state.remaining_path_x.size() && (bSkip || i < im.maxToKeep); i++) {
        Vector2d point;
        point[0] = state.remaining_path_x[i];
        point[1] = state.remaining_path_y[i];
        points.push_back(point);
        retval.x.push_back(point[0]);
        retval.y.push_back(point[1]);
    }

    if (bSkip) {
        im.prevPoints = points;
        printf("skipped - time to cover %f\n", im.trajectory.sTimeToNextJMT);
        return retval;
    }

    //localize the s/d based on the last point to keep
    double pos = state.s;
    if (im.prevPoints.size() > 0) {
        int offset = (int) (im.trajectory.sTimeToNextJMT / im.dt);
        Vector2d prevPoint = points[-offset];
        std::vector<double> frenet = im.getFrenet(state, prevPoint[0], prevPoint[1], im.waypoints_.x, im.waypoints_.y);
        pos = frenet[0];
    }

    //generate points along this path in one second
    auto jmtParams = im.generate_constraint_JMT(state, 0, 22, 10, 10);

    int targetItems = (int) (im.dT / im.dt);

    // determine optimal length
    double remT = im.dT - jmtParams.sTimeToNextJMT;
    double s_start = JMTeval(jmtParams.sJMTparams, 0) + pos;
    double s_end = JMTeval(jmtParams.sJMTparams, remT) + pos;
    double s_diff = s_end - s_start;

    // determine optimal path along JMT
    std::vector<Vector2d> optimalPoints = im.generate_path_points(im.spline_, im.waypoints_, jmtParams, points, i, s_start, s_end, im.s_max_, (int) toKeep, targetItems, false);

    // generate new scaled path points
    Waypoints scaledWaypoints = generate_scaled_waypoints_from_points(optimalPoints, s_start, s_end, im.uniformInterval, 1.0);

    // generate new localized spline
    std::vector<Vector2d> vScaledWaypoints = xyVectorsToVector2d(scaledWaypoints.x, scaledWaypoints.y);
    SplineType localizedSpline(vScaledWaypoints);

    //update waypoint distances for local spline
    std::vector<double> sValues;
    int j = i;
    for (; j < scaledWaypoints.s.size(); j++) {
        sValues.push_back(j - i);
    }
    scaledWaypoints.s = sValues;

    std::vector<Vector2d> normalizedPoints = im.generate_path_points(&localizedSpline, scaledWaypoints, jmtParams, points, i, 0, s_diff, s_diff, (int) toKeep, targetItems, false);

    for (Vector2d v : optimalPoints) {
        retval.x.push_back(v[0]);
        retval.y.push_back(v[1]);
        printf("   [%f, %f]", v[0], v[1]);
    }

    printf("complete\n");

#if 0
    printf("Used: %ld\n", used);
    printf("S: %f, Estimated S: %f, diff: %f\n", state.s, JMTeval(im.JMTparams, usedT), state.s - JMTeval(im.JMTparams, usedT));
    printf("Diff between new/old JMT: %f, normal diff: %f\n", diff, normalDiff);
    printf("Position: %f %f\n", state.x, state.y);
    //printf("Speed, Accel: %f %f\n", JMTSpeedEval(jmtParams, 0), JMTAccelEval(jmtParams, 0));
    printf("Stated Speed: %f\n", state.speed);
    printf("Closest Waypoint: %f %f\n", im.waypoints_.x[prevWaypoint], im.waypoints_.y[prevWaypoint]);
    printf("Next Waypoint: %f %f\n", im.waypoints_.x[nextWaypoint], im.waypoints_.y[nextWaypoint]);
    printf("Start: %f %f\n", retval.x[0], retval.y[0]);
    printf("End: %f %f\n\n", retval.x[retval.x.size() - 1], retval.y[retval.y.size() - 1]);
#endif

    im.prevPoints = points;
    im.trajectory = jmtParams;

    return retval;
}

std::vector<Vector2d> PathGenerator::impl::generate_path_points(SplineType* spline, Waypoints& waypoints,
                                                                const TrajectoryData& jmtParams, std::vector<Vector2d> points,
                                                                int startI, double s_start, double s_end, double s_max,
                                                                int toKeep, int maxItems, bool validate) {

    int prevWaypoint = find_waypoint_floor(waypoints.s, s_start);
    int nextWaypoint = find_waypoint_floor(waypoints.s, s_end) + 1;

    if (nextWaypoint == prevWaypoint) {
        nextWaypoint++;
    }

    if (prevWaypoint < 0) {
        prevWaypoint += waypoints.s.size();
    }

    if (nextWaypoint >= waypoints.s.size()) {
        nextWaypoint -= waypoints.s.size();
    }

    int waypointDiff = nextWaypoint - prevWaypoint;
    double s_diff = waypoints.s[nextWaypoint] - waypoints.s[prevWaypoint];

    if (s_diff <= 0) {
        s_diff = s_max -  waypoints.s[prevWaypoint];
        s_diff += waypoints.s[nextWaypoint];
    }

    if (waypointDiff <= 0) {
        waypointDiff += waypoints.s.size();
    }

    printf("prevWaypoint %d, nextWaypoint %d\n", prevWaypoint, nextWaypoint);

    std::vector<double> sv;
    int j = 0, i = startI;
    double s_rate = s_diff / waypointDiff;
    double dRemT = jmtParams.dTimeToNextJMT;
    for (j = 0; i < maxItems; i++, j++) {
        double s_delta_eval = JMTeval(jmtParams.sJMTparams, j * dt) + s_start;
        if (validate && s_delta_eval > s_end + 0.0001) {
            throw new std::exception();
        }
        double s_curr_delta = s_delta_eval - waypoints.s[prevWaypoint];
        double s = (s_curr_delta / s_rate) + prevWaypoint;
        SplineType::InterpolatedPTC curvature = spline->getCurvature(s);
        Eigen::Vector2d normal = getNormalFromCurvature(curvature, s);

        double x = curvature.position[0];
        double y = curvature.position[1];
        double d = jmtParams.currentLane * 4.0 + 2.0;

        //determine d offset
        if (dRemT > 0) {
            double d_delta = JMTeval(jmtParams.dJMTparams, jmtParams.dTimeOffsetJMT + j * dt);
            d += d_delta;
            printf("d_delta %f, j %d, offset %f, d %f\n", d_delta, j, jmtParams.dTimeOffsetJMT + j * dt, d);
            dRemT -= dt;
        }

        x += normal[0] * d;
        y += normal[1] * d;

        if (points.size() > 0) {
            Vector2d& point = points[points.size() - 1];
            double dist = distance(point[0], point[1], x, y);
            double dists = 0;
            if (sv.size() > 0) {
                dists = s_curr_delta - sv[sv.size() - 1];
                if (validate && dists > 22.f*0.02f) {
                    throw new std::exception();
                }
            }
            if (validate && dist > 23 * 0.02f) {
                throw new std::exception();
            }
        }

        Vector2d point;
        point[0] = x;
        point[1] = y;
        points.push_back(point);
        sv.push_back(s_curr_delta);
    }

    return points;
}

int PathGenerator::impl::find_waypoint_floor(std::vector<double>& waypoints, double s) {
    int waypoint = 0;
    while (waypoint < waypoints.size() -1 && waypoints[waypoint+1] <= s) {
        waypoint++;
    }
    return waypoint;
}

double PathGenerator::impl::find_closest_time(double s) {
    double t = 0;
    while (t < dT) {
        if (JMTeval(trajectory.sJMTparams, t + dt) > s) {
            break;
        }
        t += dt;
    }
    return t;
}

double PathGenerator::impl::find_target_t(std::vector<double>& waypoints, double s) {
    int prevWaypoint = find_waypoint_floor(waypoints, s);
    int nextWaypoint = prevWaypoint + 1;

    if (nextWaypoint == prevWaypoint) {
        nextWaypoint++;
    }

    if (prevWaypoint < 0) {
        prevWaypoint += waypoints.size();
    }

    if (nextWaypoint >= waypoints.size()) {
        nextWaypoint -= waypoints.size();
    }

    int waypointDiff = nextWaypoint - prevWaypoint;
    double s_diff = waypoints[nextWaypoint] - waypoints[prevWaypoint];

    if (s_diff < 0) {
        s_diff = s_max_ -  waypoints[prevWaypoint];
        s_diff += waypoints[nextWaypoint];
    }

    if (waypointDiff < 0) {
        waypointDiff += waypoints.size();
    }

    double s_curr = s - waypoints[prevWaypoint];
    double s_rate = s_diff / waypointDiff;

    double s_base = (s_curr / s_rate) + prevWaypoint;

    return s_base;
}

TrajectoryData PathGenerator::impl::generate_constraint_JMT(VehicleState state, double pos, double max_velocity, double max_accel, double max_jerk) {

    TrajectoryData retval = trajectory;

    // number of time steps that have been consumed
    // number of time steps that remain that should be kept
    unsigned long toKeep = (unsigned long) fmin(maxToKeep, state.remaining_path_x.size());
    // elapsed time to the new 'reference point', the origin of the new JMT for this update
    double deltaT = - retval.sTimeToNextJMT + toKeep * retval.dt;

    //use the previous JMT and final reference point to determine where the
    double speed =  JMTSpeedEval(trajectory.sJMTparams, deltaT);
    double accel = JMTAccelEval(trajectory.sJMTparams, deltaT);

    // determine JMT using start and end state

    // amount of time necessary to generate a full prediction horizon
    double remT = dT - toKeep * trajectory.dt;
    if (!(remT > toKeep * trajectory.dt)) {
        throw new std::exception();
    }

    double furthest_s = pos + max_velocity * remT;
    auto bestCurveS = permuteJMT(pos, speed, accel, furthest_s, max_velocity, max_velocity, max_accel, max_jerk, remT, dt);

    if (bestCurveS.JMTparams.size() == 0) {
        throw new std::exception();
    }

    //check for collisions
    auto overlaps = check_overlap(state.d, state.sensor_state, bestCurveS.JMTparams, remT, dt);
    if (!overlaps.empty() && retval.dTimeToNextJMT <= 0) {

        //find closest colliding object
        auto collisions = check_collisions(state.sensor_state, overlaps, state.d, bestCurveS.JMTparams, remT, dt);

        if (!collisions.empty()) {
            CollisionBoundingBox collision;
            if (!collisions.empty())
                collision = collisions.front();

            if (collision.id != -1) {

                //determine if there's a free adjacent lane
                auto overlapsLeft = check_overlap(fmax(0, state.d - 4.0), state.sensor_state, bestCurveS.JMTparams, remT, dt);
                auto overlapsRight = check_overlap(fmin(10, state.d + 4.0), state.sensor_state, bestCurveS.JMTparams, remT, dt);
                int currentLane = (int) (state.d / 4.0);
                int newLane = currentLane;

                if (overlapsLeft.empty()) {
                    newLane = (int) fmax(0, state.d - 4.0) / 4;
                } else if (overlapsRight.empty()) {
                    newLane = (int) fmin(10.0, state.d + 4.0) / 4;
                }

                if (currentLane != newLane) {
                    //generate JMT 'd' trajectory
                    double startD = 0; //state.d - (currentLane * 4.0 + 2.0);
                    double endD = (newLane * 4.0 + 2.0) - (currentLane * 4.0 + 2.0);
                    auto bestCurveD = permuteJMT(startD - endD, 0, 0, 0, 0, max_velocity, max_accel,
                                                 max_jerk, remT, dt, false);
                    if (bestCurveD.JMTparams.empty()) {
                        throw new std::exception();
                    }

                    printf("changing lanes: lane %d, startD %f, endD %f\n", newLane, startD, endD);

                    retval.dJMTparams = bestCurveD.JMTparams;
                    retval.dTimeOffsetJMT = 0;
                    retval.dTimeToNextJMT = remT;
                    retval.currentLane = newLane;

                    //determine new max speed
                    bestCurveS = permuteJMT(pos, speed, accel, collision.start, max_velocity, max_velocity, max_accel,
                                            max_jerk, remT, dt);
                    collisionState = collision;
                    if (bestCurveS.JMTparams.empty()) {
                        throw new std::exception();
                    }

                } else {
                    printf("collision detected: id %d, speed %f pos %f\n", collision.id, collision.speed,
                           collision.start);
                    //determine new max speed
                    bestCurveS = permuteJMT(pos, speed, accel, fmax(collision.start - minimumFollowDistance, pos), collision.speed, max_velocity, max_accel,
                                           max_jerk, remT, dt);
                    collisionState = collision;
                    if (bestCurveS.JMTparams.empty()) {
                        throw new std::exception();
                    }
                }
            }

        }
        //check if the previous collision state is still valid
        else if (does_collide(overlaps, collisionState.id)) {
            //continue following speed
            collisionState.speed = state.sensor_state[collisionState.id].speed;
            printf("following vehicle: id %d, speed %f\n", collisionState.id, collisionState.speed);
            bestCurveS = permuteJMT(pos, speed, accel, fmax(furthest_s - minimumFollowDistance, pos), collisionState.speed, max_velocity, max_accel,
                                   max_jerk, remT, dt);
        } else {
            //invalidate
            collisionState.id = -1;
        }

        if (bestCurveS.JMTparams.empty()) {
            throw new std::exception();
        }
    } else {
        collisionState.id = -1;
    }

    retval.sJMTparams = bestCurveS.JMTparams;
    retval.sTimeToNextJMT = maxToKeep * trajectory.dt;
    retval.sTimeOffsetJMT = 0;

    double dDelta = JMTeval(bestCurveS.JMTparams, 0) - JMTeval(trajectory.sJMTparams, deltaT - trajectory.dt);

    return retval;
}

VecCollisionBoundingBox PathGenerator::impl::check_overlap(double d, const std::vector<SensorVehicleState>& sensor_state, const std::vector<double>& params, double dT, double dt) {
    VecCollisionBoundingBox retval;
    std::vector<CollisionBoundingBox> boundingBoxesCar = calculateLaneBoundingBox(d, JMTeval(params, 0), JMTSpeedEval(params, 0), dT, 4.0);
    for (SensorVehicleState s : sensor_state) {
        std::vector<CollisionBoundingBox> boundingBoxes = calculateLaneBoundingBoxForSensorVehicleState(s, dT, 4.f);

        bool collide = does_overlap(boundingBoxesCar, boundingBoxes);
        if (collide) {
            retval.push_back(boundingBoxes[0]);
        }
    }
    return retval;
}

VecCollisionBoundingBox PathGenerator::impl::check_collisions(const std::vector<SensorVehicleState>& vehicles,
                                                              const VecCollisionBoundingBox& boxes,
                                                              double d,
                                                              const std::vector<double>& params,
                                                              double dT,
                                                              double dt) {
    VecCollisionBoundingBox retval;
    for (const CollisionBoundingBox& box : boxes) {
        bool doesCollide = false;
        double smallestS = box.end;
        CollisionBoundingBox collisionBox = box;
        //determine exact moment of collision
        const SensorVehicleState& other_vehicle = vehicles[box.id];
        double collisionS = calculateCollisionMoment(other_vehicle, params, d, dT, dt, 2.f, 5.f);
        if (collisionS != -1) {
            if (collisionS >= 0 && smallestS >= collisionS) {
                collisionBox = box;
                collisionBox.start = collisionS;
                collisionBox.speed = box.speed;
                retval.push_back(collisionBox);
            }
        }
    }
    return retval;
}

PathGenerator::impl::impl() : trajectory({}) {
    trajectory.dTimeToNextJMT = 0;
    trajectory.dTimeOffsetJMT = 0;
    trajectory.sTimeToNextJMT = 0;
    trajectory.sTimeOffsetJMT = 0;
    trajectory.currentLane = 1;
    trajectory.dt = 0;
    trajectory.sJMTparams = std::vector<double>(6);
    trajectory.dJMTparams = std::vector<double>(6);
}


PathGenerator::impl::~impl() {
    delete spline_;
}
