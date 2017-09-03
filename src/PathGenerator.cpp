//
// Created by Jose Rojas on 7/23/17.
// Copyright © 2017, Jose Luis Rojas
// All Rights Reserved.
//

#include "PathGenerator.h"
#include <random>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Geometry"
#include "SplineLibrary/spline_library/splines/natural_spline.h"

typedef Vector<2, double> Vector2d;

//Natural Cubic Splines are used from the SplineLibrary
typedef Spline<Vector2d,double> SplineCommonType;
typedef NaturalSpline<Vector2d,double> SplineType;
typedef LoopingNaturalSpline<Vector2d,double> LoopingSplineType;

// Global definitions
static const int MAX_LANES = 3;
static const double CAR_WIDTH = 2.75;
static const double CAR_LENGTH = 4.5;
static const double MAX_RELATIVE_VELOCITY = 10.0;
static const double SPEED_LIMIT = 22.286;
static const double SPEED_LIMIT_VALIDATION = 22.292;
static const double COLLISION_HORIZON = 8.0;
static const double COLLISION_BUFFER_RANGE = 1.0;
static const double MINIMUM_FOLLOW_DISTANCE = 10.0;
static const double COLLISION_DISTANCE_THRESHOLD = 80.0;
static const double PASSING_SPEED_INCREMENT = 5.0;
static const double MAX_ACCEL_JERK = 10;
static const double D_MAX_VELOCITY = 4.0;
static const double D_MAX_ACCEL_JERK = 3.0;
static const double LANE_CHANGE_TIME = 3.0;
static const double LANE_CHANGE_TIME_EXTENDED = 5.0;

/*
 * Helper Routines
 */

static Eigen::Rotation2D<double> g_rotNormal(-90.f * M_PI / 180.f);

extern double distance(double x1, double y1, double x2, double y2);

/*
 * Speed and position evaluators
 */

double SPEEDeval(const std::vector<double>& params, double T) {
    double t2 = T;
    return params[1] + 2 * params[2] * t2;
}

double POSeval(const std::vector<double>& params, double T) {
    double t2 = T * T;
    double val = params[0] + params[1] * T + params[2] * t2;
    return val;
}

/*
 * Acceleration equations
 */

double tangentialAccel(const Eigen::Vector2d& v, const Eigen::Vector2d& a) {
    return v.dot(a) / v.norm();
}

double normalAccel(const Vector2d& v, const Vector2d& a) {
    Eigen::Vector3d vv, aa;
    vv << v[0], v[1], 0;
    aa << a[0], a[1], 0;
    return vv.cross(aa).norm() / vv.norm();
}

/*
 * Normal calculation from spline curvature structure
 */

Eigen::Vector2d getNormalFromCurvature(SplineCommonType::InterpolatedPTC& curvature, double t) {
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

/*
 * K curvature (1/R) calculation from spline curvature structure
 */

double getCurvatureScalerFromCurvature(SplineCommonType::InterpolatedPTC& curvature) {
    Vector2d cx = curvature.curvature;
    Vector2d v = curvature.position;
    Vector2d tx = curvature.tangent;
    double x_dot = tx[0];
    double y_dot = tx[1];
    double x_ddot = cx[0];
    double y_ddot = cx[1];

    return (x_dot * y_ddot - y_dot * x_ddot) / pow(x_dot * x_dot + y_dot * y_dot, 1.5);
}

Waypoints generate_scaled_waypoints_from_points(const std::vector<Vector2d>& xv, double s, double s_max, double sourceInterval, double destInterval) {

    SplineType spline(xv);

    // the original waypoints are not uniformly distributed and our cubic hermite spline requires uniformly distributed
    // points - regenerate a set of waypoints with a uniform 's' distribution

    Waypoints newWaypoints;

    double t = 0, err = 0;

    while (s <= s_max) {

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

    //printf("Total error = %f\n", err);

    return newWaypoints;
}

/*
 * Arc length calculation - sums of the Euclidean distances of a set of points
 */

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

/*
 * Conversion from x, y vectors to a vector of Vector2d
 */

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

/*
 * Permutation helper function. Does a uniform optimized search from a center point, outward across a range of values.
 * The lambda function serves as the implementation for the search.
 * Returning true from the lambda will break the loop early.
 */

bool permuteParameter(double center, double offset, double inc, bool useHigh, bool useLow, const std::function <bool (double)>& f) {

    double p = 0;
    int factor = 0;
    offset = fabs(offset);
    auto diff = ((int)useHigh + (int) useLow) * offset / inc;

    while (factor <= diff) {

        double sign = useHigh && useLow ? (factor % 2) == 0 ? -1.f : 1.f : (useHigh) ? 1.0 : (useLow) ? -1.0 : 0.0;
        int mult = useHigh && useLow ? ((factor + 1) / 2) : factor;

        p = center + mult * sign * inc;

        if (f(p)) {
            return true;
        }

        factor++;
    }

    return false;
}

// Collision data structure - contains information about a potential collision with a vehicle
typedef struct {
    int id;
    int lane;
    double time;
    double position;
    double speed;
} Collision;

// JMT - Jerk Minimal Trajectory class

class JMT {

private:

    std::vector<double> params;
    double Tmax;
    double _cost;

public:

    JMT() : params(std::vector<double>(0)) {
        _cost = 0;
        Tmax = 0;
    }

    JMT& init(std::vector< double> start, std::vector <double> end, double T)
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

        > jmt( [0, 10, 0], [10, 10, 0], 1)
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

        this->params = {start[0],start[1], 0.5 * start[2], x(0), x(1), x(2)};
        this->Tmax = T;
        this->_cost = 0;

        return *this;
    }

    bool isDefined() { return !params.empty(); }

    double position(double T) const {
        double t = fmin(T, Tmax);
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;
        double val = params[0] + params[1] * t + params[2] * t2 + params[3] * t3 + params[4] * t4 + params[5] * t5;
        return val + (T > Tmax ? speed(T) * (T - Tmax) : 0);
    }

    double speed(double T) const {
        T = fmin(T, Tmax);
        double t2 = T;
        double t3 = t2 * T;
        double t4 = t3 * T;
        double t5 = t4 * T;
        return params[1] + 2 * params[2] * t2 + 3 * params[3] * t3 + 4 * params[4] * t4 + 5 * params[5] * t5;
    }

    double accel(double T) const {
        double t3 = T;
        double t4 = t3 * T;
        double t5 = t4 * T;
        return T > Tmax ? 0 : 2 * params[2] + 6 * params[3] * t3 + 12 * params[4] * t4 + 20 * params[5] * t5;
    }

    double jerk(double T) const {
        T = fmin(T, Tmax);
        double t4 = T;
        double t5 = t4 * T;
        return T > Tmax ? 0 : 6 * params[3] + 24 * params[4] * t4 + 60 * params[5] * t5;
    }

    double T() const { return Tmax; }

    void cost(double c) { _cost = c; }

    double cost() { return _cost; }
};

/* Composite Trajectory Segment Class - a wrapper around the JMT to assist with searches of valid JMTs. Also contains
 * time management data about the age of the trajectory. */
class CTS {

private:

    JMT _jmt;
    double timeOriginJMT; //Time offset from the 'origin' reference time
    double timeDeltaJMT; //Time passed since the JMT was created

public:
    CTS() {
        timeOriginJMT = 0;
        timeDeltaJMT = 0;
    }

    bool validateJMT(const JMT& jmtCurve,
                     std::vector<double> limits,
                     double dT, double dt) const {
        //does it violate acceleration
        bool failedConstraint = false;
        for (double t = 0; t < dT && !failedConstraint; t += dt) {
            double accel = jmtCurve.accel(t);
            if (fabs(accel) > limits[1]) {
                failedConstraint = true;
                //printf("accel violation: %f\n", accel);
                break;
            }
        }

        //does it violate speed
        for (double t = 0; t < dT && !failedConstraint; t += dt) {
            double speed = jmtCurve.speed(t);
            if (fabs(speed) > limits[0]) {
                failedConstraint = true;
                //printf("speed violation: %f\n", speed);
                break;
            }
        }

        //does it violate jerk
        for (double t = 0; t < dT && !failedConstraint; t += dt) {
            double jerk = jmtCurve.jerk(t);
            if (fabs(jerk) > limits[2]) {
                failedConstraint = true;
                //printf("jerk violation: %f\n", jerk);
                break;
            }
        }

        return !failedConstraint;
    }

    double cost_function(std::vector<double> target,
                         std::vector<double> estimate) const {
        // we mostly care about the speed
        const double SPEED_WEIGHT = 5.0;
        double posDiff = (target[0] - estimate[0]);
        double speedDiff = (target[1] - estimate[1]);
        double accelDiff = (target[2] - estimate[2]);
        return speedDiff * speedDiff * SPEED_WEIGHT + posDiff * posDiff + accelDiff * accelDiff;
    }

    /*
     * Guassian search for valid JMTs with a set of thresholds.
     */

    std::vector<JMT> searchJMTs(std::vector<double> start,
                                std::vector<double> meanEnd,
                                std::vector<double> stdDeviation,
                                std::vector<double> limits,
                                int numberOfStateSamples,
                                double dT, double dt, double dvT, bool verbose= false) const  {
        //permute states
        std::map<double, JMT> map;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> dist[3] = {
            std::normal_distribution<>(meanEnd[0], stdDeviation[0]),
            std::normal_distribution<>(meanEnd[1], stdDeviation[1]),
            std::normal_distribution<>(meanEnd[2], stdDeviation[2])
        };

        permuteParameter(dT, dvT, dt, true, true, [this, &limits, &start, &numberOfStateSamples, &meanEnd, &dist, &gen, &dT, &dt, &verbose, &map](double timeInterval)->bool {

            if (verbose)
                printf("searching within time interval %f\n", timeInterval);

            if (timeInterval > 0) {
                for (int i = 0; i < numberOfStateSamples; i++) {
                    double v = limits[0] + 1, acc = limits[1] + 1, pos;
                    do pos = dist[0](gen); while (pos < 0);
                    while (v > limits[0] || v < 0)
                        v = dist[1](gen);
                    while (fabs(acc) > limits[1])
                        acc = dist[2](gen);

                    if (v > limits[0])
                        throw std::exception();
                    if (fabs(acc) > limits[1])
                        throw std::exception();

                    std::vector<double> end{pos, v, acc};

                    JMT params = JMT().init(start, end, timeInterval);

                    if (verbose)
                        printf("    start { pos %f, v %f, accl %f}, end { pos %f, v %f, accl %f}\n", start[0], start[1], start[2], pos, v, acc);

                    if (this->validateJMT(params, limits, timeInterval, dt)) {
                        if (verbose) printf("    found!\n");
                        params.cost(cost_function(meanEnd, end));
                        // if JMTs share the lowest cost, they will be overwritten, but that's generally okay for this purpose
                        map.insert(std::make_pair(params.cost(), params));
                    }

                }

                return false;
            }

            return false;

        } );

        std::vector<JMT> retval;
        for (auto it : map) {
            retval.push_back(it.second);
        }

        return retval;
    }

    /*
     * Uniform search for valid JMTs. This method of search is better optimized when a searching for specific values around a center point.
     */

    std::vector<JMT> searchJMTSpace(std::vector<double> start, double final_s, double final_velocity, std::vector<double> limits, double dT, double dt, bool bPermute = true) const {
        JMT bestCurve;

        permuteParameter(0.0, dt * 10, dt, bPermute, bPermute,
            [this, &bestCurve, &final_s,&start, &limits, &dT, &dt, &final_velocity, &bPermute](double s_ddot)->bool {

                double s_dot_inc = fabs(fmax(dt * 10, (final_velocity - start[1]) / 10));

                return permuteParameter(final_velocity, final_velocity - start[1], s_dot_inc, false, bPermute,
                    [this, &bestCurve, &final_s, &start, &limits, &s_ddot, &dT, &dt, &bPermute](double s_dot)->bool {

                     return permuteParameter(final_s, final_s - start[0], dt, bPermute, bPermute,
                        [this, &bestCurve, &start, &limits, &s_dot, &s_ddot, &dT, &dt](double s)->bool {

                        std::vector<double> end{s, s_dot, s_ddot};

                        JMT params = JMT().init(start, end, dT);

                        if (this->validateJMT(params, limits, dT, dt)) {
                            bestCurve = params;
                            //printf("jmt final (pos %f, speed %f, accel %f)\n", s, s_dot, s_ddot);
                            return true;
                        }

                        return false;
                    });

                });

            });

        //XXX Cost is not considered. Use the first result.
        if (bestCurve.isDefined())
            return {bestCurve};

        return {};
    }

    /* JMT setup */

    void defineJMT(const JMT& jmt, double timeToLive) {
        this->_jmt = jmt;
        this->timeOriginJMT = -timeToLive;
        this->timeDeltaJMT = 0;
    }

    /* Update the time properties based on time that has passed within the simulation. */

    void updateTime(double dT) {
        timeOriginJMT += dT;
        timeDeltaJMT += dT;
    }

    /* Remaining time to 'live'. The lifetime of the CTS. */
    double timeToLive() const { return -timeOriginJMT; }

    /* Time passed since the origin reference time.  */
    double originTime() const { return timeOriginJMT; }

    /* Time since creation. */
    double deltaTime() const { return timeDeltaJMT; }

    const JMT& jmt() const { return this->_jmt; }

};

/* Frenet Trajectory Segment - the combined structure to represent a frenet trajectory with S and D JMTs (CTSs) */
class FTS {
private:

    CTS _dCTS;
    CTS _sCTS;
    int _toKeep;
    double _sOffset;
    double _dt;

public:

    FTS() {
        _toKeep = 0;
        _sOffset = 0;
        _dt = 0;
    }

    void init(double max_velocity, double max_accel, double max_jerk, double dt) {
        JMT jmt = JMT().init({0,0,0}, {0,0,0}, dt);
        _dCTS.defineJMT(jmt, _dCTS.timeToLive());
        _sCTS.defineJMT(jmt, _sCTS.timeToLive());
        _dt = dt;
    }

    const JMT& d() const { return _dCTS.jmt(); }
    const JMT& s() const { return _sCTS.jmt(); }
    const CTS& sCTS() const { return _sCTS; }
    const CTS& dCTS() const { return _dCTS; }
    double dt() const { return _dt; }
    double sOffset() const { return _sOffset; }
    int toKeep() const { return _toKeep; }

    void updateTime(double dT) {
        _dCTS.updateTime(dT);
        _sCTS.updateTime(dT);
    }

    void d(const JMT& d, double dT) {
        _dCTS.defineJMT(d, dT);
    }

    void s(const JMT& s, double dT, double sOffset, int toKeep) {
        _sCTS.defineJMT(s, dT);
        _toKeep = toKeep;
        _sOffset = sOffset;
    }

};

/* Vehicle class: represents the trajectory and dimensions of a vehicle */

class Vehicle {
private:

    int _id = -1;
    FTS trajectory;
    double width, _length;

public:

    Vehicle() {
        width = _length = 0;
    }

    void initWithFTS(const FTS& fts, double width, double length) {
        this->width = width;
        this->_length = length;
        this->trajectory = fts;
    }

    void initWithSensorVehicleState(SensorVehicleState state, double width, double length, double dT, double timeOffset) {

        this->_id = state.id;
        this->width = width;
        this->_length = length;

        std::vector<double> orig_s = {state.s, state.speed, 0};
        std::vector<double> start_s = {POSeval(orig_s, timeOffset), state.speed, 0};
        std::vector<double> end_s = {POSeval(start_s, dT + timeOffset), state.speed, 0};

        //XXX assume cars do not switch lanes for now
        std::vector<double> start_d = {state.d, 0, 0};

        JMT sJMT = JMT().init(start_s, end_s, dT);
        JMT dJMT = JMT().init(start_d, start_d, dT);

        trajectory.init(state.speed, 0, 0, dT);
        trajectory.s(sJMT, dT, 0, 0);
        trajectory.d(dJMT, dT);
    }

    int id() const { return _id; }

    double length() const { return _length; }

    /* Calculates the moment of collision from the reference time. The vehicle bounding boxes are evaulated at
     * every time step in a particular range.
     * */

    double collisionTime(const Vehicle& v, double dT, double range) const {
        double t = 0;
        double start = trajectory.s().position(t) + trajectory.sOffset();
        double s = v.trajectory.s().position(t) + v.trajectory.sOffset(), d = 0;
        double hlw = width / 2.0;

        //determine lowest 's' vehicle
        const Vehicle& lowest = (s < start) ? v : *this;
        const Vehicle& other = (s < start) ? *this : v;
        double rr = range + other.length();

        while (t <= dT) {
            s = lowest.trajectory.s().position(t) + lowest.trajectory.sOffset();
            d = lowest.trajectory.d().position(t);
            double vs = other.trajectory.s().position(t) + other.trajectory.sOffset();
            double td = other.trajectory.d().position(t);
            bool right = (td + hlw >= d - hlw) && (td + hlw <= d + hlw);
            bool left = (td - hlw >= d - hlw) && (td - hlw <= d + hlw);
            bool ahead = s > vs - rr;
            if (ahead && (right || left)) {
                if (t == 0) {
                    t = t;
                }
                return t;
            }
            t += v.trajectory.dt();
        }
        return -1;
    }

    /* Overlaps are simpler calculations that collisions. It simply checks if the end points of a set of vehicle
     * trajectories overlap.
     * */

    bool does_overlap(const Vehicle& v, double dT, double laneWidth) const {

        //assumption: d represents offset position from left to right

        double s1[2] = {v.trajectory.s().position(0) + v.trajectory.sOffset(),
                        v.trajectory.s().position(dT) + v.trajectory.sOffset()};
        double s2[2] = {trajectory.s().position(0) + trajectory.sOffset(),
                        trajectory.s().position(dT) + trajectory.sOffset()};
        double d1[2] = {v.trajectory.d().position(0),
                        v.trajectory.d().position(dT)};
        double d2[2] = {trajectory.d().position(0),
                        trajectory.d().position(dT)};

        double vw1 = v.width / 2;
        double vw2 = width / 2;
        double maxd1 = fmax(d1[0], d1[1]);
        double mind1 = fmin(d1[0], d1[1]);
        double maxd2 = fmax(d2[0], d2[1]);
        double mind2 = fmin(d2[0], d2[1]);

        bool doesLaneChange = ((int) (d1[0] / laneWidth) != (int) (d1[1] / laneWidth)) ||
                              ((int) (d2[0] / laneWidth) != (int) (d2[1] / laneWidth));
        bool doesStartBehind = (s1[0] < s2[0]);
        bool doesStartOverlap1 = s2[0] >= s1[0] && s2[0] <= s1[1];
        bool doesEndOverlap1 = s2[1] >= s1[0] && s2[1] <= s1[1];
        bool doesLaneOverlapRight = maxd1 + vw1 >= mind2 - vw2 && maxd1 + vw1 <= maxd2 + vw2;
        bool doesLaneOverlapLeft = mind1 - vw1 >= mind2 - vw2 && mind1 - vw1 <= maxd2 + vw2;

        bool overlap = (doesStartOverlap1 || doesEndOverlap1)
                   && (doesLaneOverlapLeft || doesLaneOverlapRight)
                   && (doesStartBehind || doesLaneChange);

        return overlap;
    }

    const FTS& fts() const { return trajectory; }

};

/* Path Generator private implementation */

class PathGenerator::impl {

private:
    double s_max_;
    Waypoints waypoints_;
    LoopingSplineType* spline_;
    FTS trajectory;
    double dT;
    double dt;
    double uniformInterval;
    double laneWidth;
    std::vector<Vector2d> prevPoints;
    Collision lastCollision;
public:

    impl(Waypoints waypoints, double s_max) : trajectory({}) {
        s_max_ = s_max;
        uniformInterval = 1.f;

        // Generate a normalized set of spline waypoints so that the parameter of a spline matches the 's' arc length
        // this will ensure that our frenet 's' distance matches the actual 2D distance traveled in Euclidean space.
        std::vector<Vector2d> xv_lane;
        for (int p = 0; p < waypoints.x.size(); p++) {
            Vector2d v;
            v[0] = waypoints.x[p];
            v[1] = waypoints.y[p];
            xv_lane.push_back(v);
        }
        //enable looping
        xv_lane.push_back(xv_lane[0]);

        Waypoints newWaypoints = generate_scaled_waypoints_from_points(xv_lane, 0, s_max, uniformInterval,
                                                                       uniformInterval);
        xv_lane = xyVectorsToVector2d(newWaypoints.x, newWaypoints.y);

        spline_ = new LoopingSplineType(xv_lane);
        waypoints_ = newWaypoints;

        dT = 0; //this will be scaled dynamically based on speed
        dt = 0.02;
        laneWidth = 4.0;
        trajectory.init(SPEED_LIMIT, MAX_ACCEL_JERK, MAX_ACCEL_JERK, dt);
    }

    ~impl() {
        delete spline_;
    }

    int find_waypoint_floor(std::vector<double>& waypoints, double s) {
        int waypoint = 0;
        while (waypoint < waypoints.size() -1 && waypoints[waypoint+1] <= s) {
            waypoint++;
        }
        return waypoint;
    }

    /* Path generation implementation */

    PathPoints generate_path(VehicleState state) {

        //localize s/d according to the internal waypoints/spline
        std::vector<double> frenet = getFrenet(state.s, state.x, state.y, waypoints_, *spline_, uniformInterval);
        state.s = frenet[0];
        state.d = frenet[1];

        static double totalTime = 0;

        if (state.speed < 18) {
          dT = 3.5;
        } else {
          dT = LANE_CHANGE_TIME;
        }

        int used = (int) fmax(0, (int) prevPoints.size() - (int) state.remaining_path_x.size());
        double deltaT = used * trajectory.dt();
        double lengthDistTraveled = arcLength(prevPoints, 0, used);


        //reduce time
        trajectory.updateTime(deltaT);
        totalTime += deltaT;

        double offsetTime = trajectory.sCTS().originTime();

        printf("used %d, time delta %f, jmt time offset %f, jmt time remaining %f, distance traveled %f, speed %f, accel %f\n",
               used,
               deltaT,
               offsetTime,
               trajectory.sCTS().timeToLive(),
               lengthDistTraveled,
               trajectory.s().speed(offsetTime),
               trajectory.s().accel(offsetTime)
        );

        bool bSkip = trajectory.sCTS().timeToLive() > 0.0001;

        // add previous points
        PathPoints retval;
        std::vector<Vector2d> points;
        int i = 0;
        for (; i < state.remaining_path_x.size() && bSkip; i++) {
            Vector2d point = prevPoints[used + i];
            points.push_back(point);
            retval.x.push_back(point[0]);
            retval.y.push_back(point[1]);
        }

        if (bSkip) {
            prevPoints = points;
            //printf("skipped - time to cover %f\n", trajectory.sCTS().timeToLive());
            return retval;
        }

        // For debugging purposes
        if (totalTime > 5.0) {
            //do state.forceLane = (int)(state.d / laneWidth) == 2 ? 0 : 2; while (state.forceLane == (int)(state.d / laneWidth));
            totalTime = -5.0;
        }

        /* update the other vehicle state */
        for (SensorVehicleState& v : state.sensor_state) {
            // determine frenet position and tangential speed of vehicles (based on road spline)
            std::vector<double> frenet = getFrenet(v.s, v.x, v.y, waypoints_, *spline_, uniformInterval);
            v.s = frenet[0];
            v.d = frenet[1];
            Vector2d tangent = spline_->getTangent(v.s).tangent;
            Vector2d speed;
            speed[0] = v.v_x;
            speed[1] = v.v_y;
            v.speed = tangent.dotProduct(speed, tangent);
        }

        //generate points along this path in one second
        auto jmtParams = updateTrajectory(trajectory, state, SPEED_LIMIT, 10, 10);

        // add points to keep
        for (i = 0; i < jmtParams.toKeep(); i++) {
            Vector2d point = prevPoints[used + i];
            points.push_back(point);
        }

        int currentLane = 0; //(int) round(jmtParams.currentLane);
        int targetItems = (int) (dT / dt);

        //localize the s/d based on the last point to keep
        double currPos = state.s;
        double originPos = currPos;
        double dOffset = state.d;
        if (prevPoints.size() > 0) {

            /* Debugging purposes */
            Vector2d point = prevPoints[used-1];
            double posFrenet = getFrenet(state.s, point[0], point[1], waypoints_, *spline_, uniformInterval, false)[0];
            Vector2d pointAhead;
            double posFrenetAhead;
            pointAhead = points[points.size() - 1];
            posFrenetAhead = getFrenet(state.s, pointAhead[0], pointAhead[1], waypoints_, *spline_, uniformInterval)[0];

            //printf("Current x, y [%f, %f], state x, y [%f, %f] \n", point[0], point[1], state.x, state.y);
            //printf("pos %f, currPos %f, posFrenet %f, posFrenetAhead %f, posState %f, diffFrenet %f, diffCurrFrenet %f, diffFrenetAhead %f, diffState %f\n", pos, currPos, posFrenet, posFrenetAhead, state.s, pos - posFrenet, currPos - posFrenet, pos - posFrenetAhead, pos - state.s);
            //printf("old jmt sOffset %f, new jmt sOffset %f\n", trajectory.sOffset(), jmtParams.sOffset());

            printf("Current x, y [%f, %f], state x, y [%f, %f] \n", point[0], point[1], state.x, state.y);
            printf("curPos %f, posFrenet %f, state.s %f, originPos %f, originFrenet %f \n", currPos, posFrenet, state.s, originPos, posFrenetAhead);
            printf("old jmt sOffset %f, new jmt sOffset %f\n", trajectory.sOffset(), jmtParams.sOffset());

            if (distance(point[0], point[1], state.x, state.y) > 0.001) {
                throw std::exception();
            }

            if (fabs(point[0] - state.x) > 0.001 || fabs(point[1]- state.y) > 0.001) {
                throw std::exception();
            }

            if (fabs(posFrenet - currPos) > 0.001) {
                throw std::exception();
            }

            originPos = posFrenetAhead;

            /* Debugging
            if (fabs(pos - state.s) > 20) {
                posFrenetAhead = getFrenet(state.s, pointAhead[0], pointAhead[1], waypoints_[currentLane], *spline_[currentLane], uniformInterval)[0];
                throw std::exception();
            }*/
        }

        // determine optimal length
        double remT = dT - jmtParams.sCTS().timeToLive();
        double s_start = jmtParams.s().position(0) + originPos;
        double s_end = jmtParams.s().position(remT) + originPos;
        double s_diff = s_end - s_start;

        //printf("Pos = %f, s_start %f, s_end %f, s_diff %f\n", pos, s_start, s_end, s_diff);

        // determine optimal path along jmt using original road waypoints
        std::vector<Vector2d> newPoints;
        std::vector<Vector2d> optimalPoints = generate_path_points(spline_, waypoints_,
                                                                   jmtParams, newPoints, (int) newPoints.size(), 1 * jmtParams.dt(),
                                                                   s_start, s_end, s_max_, 0, true,
                                                                   targetItems, false);


        double optimalArcLength = arcLength(optimalPoints, 0, optimalPoints.size());

        if (optimalArcLength == 0) {
            printf("Error: arc length for trajectory is 0!");
            throw std::exception();
        }

        // add keep points to optimal set
        std::vector<Vector2d> appendedPoints(points);
        appendedPoints.insert(appendedPoints.end(), optimalPoints.begin(), optimalPoints.end());

        double appendedArcLength = arcLength(appendedPoints, 0, appendedPoints.size());

        // generate new scaled path points with appended waypoints from previous curve
        Waypoints scaledWaypoints = generate_scaled_waypoints_from_points(appendedPoints, 0, appendedArcLength, uniformInterval, 1.0);

        // generate new localized spline
        std::vector<Vector2d> vScaledWaypoints = xyVectorsToVector2d(scaledWaypoints.x, scaledWaypoints.y);
        SplineType localizedSpline(vScaledWaypoints);

        printf("To Keep points\n");
        for (Vector2d point : points) {
            printf(" [%f, %f], ", point[0], point[1]);
        }
        printf("\n");

        printf("\nOptimal points\n");
        for (Vector2d point : optimalPoints) {
            printf(" [%f, %f], ", point[0], point[1]);
        }
        printf("\n");

        printf("\nScaled waypoints\n");
        for (Vector2d point : vScaledWaypoints) {
            printf(" [%f, %f], ", point[0], point[1]);
        }
        printf("\n");

        double toKeepArcLength = arcLength(points, 0, points.size());
        double scaledWayPointsArcLength = arcLength(vScaledWaypoints, 0, vScaledWaypoints.size());

        // the scaledWaypoints include the 'toKeep' points which have already been scaled, so to generate new scaled points,
        // the offset into the spline must be toKeepArcLength. Also to prevent overlap with the last keep point, the time offset
        // is shifted by one time unit.
        points = generate_path_points(&localizedSpline, scaledWaypoints, jmtParams, points, i, 1 * jmtParams.dt(),
                                         toKeepArcLength, scaledWayPointsArcLength, scaledWayPointsArcLength, 0,
                                         false, targetItems, true);
        //points = im.generate_path_points(im.spline_[currentLane], im.waypoints_[currentLane], jmtParams, points, i, 0, s_start, s_end, im.s_max_, d_start, targetItems, true);

        double scaledArcLength = arcLength(points, 0, points.size());

        printf("\nScaled path points\n");
        for (Vector2d point : points) {
            retval.x.push_back(point[0]);
            retval.y.push_back(point[1]);
            printf(" [%f, %f], ", point[0], point[1]);
        }
        printf("\n");

        //printf("s_diff %f, toKeepArcLength %f, optimalArcLength %f, appendedArcLength %f, scaled waypoint arcLength %f, scaledArcLength %f\n", s_diff, toKeepArcLength, optimalArcLength, appendedArcLength, scaledWayPointsArcLength, scaledArcLength);

        //printf("complete\n");

        //validate frenet coordinates
        /*
        int pts = 0;
        for (auto point : points) {
            double d = jmtParams.d().position(pts * dt);
            double s = state.s + jmtParams.s().position(pts * dt);
            double dEst = getFrenet(s, point[0], point[1], waypoints_, *spline_, uniformInterval)[1];
            if (fabs( dEst - d) > 0.5) {
                pos = pos;
            }
            pts++;
        }
         */


        prevPoints = points;
        trajectory = jmtParams;

        return retval;
    }

    /* Update the trajectory information (s, d) by creating a continuous curve from previous updates along with
     * considering future collisions with vehicles.
     * */

    FTS updateTrajectory(const FTS& currFTS, VehicleState state, double max_velocity, double max_accel, double max_jerk) {

        FTS nextFTS = currFTS;

        double currTime = currFTS.sCTS().originTime();
        double currPos = currFTS.s().position(currTime);
        double currPosAbs = currFTS.s().position(currTime) + currFTS.sOffset();
        double currAbsPos = currPos + currFTS.sOffset();
        double currD = currFTS.d().position(currTime);

        // iterate to find how many points to keep
        int toKeep = 0;
        int remainingPoints = (int) state.remaining_path_x.size();
        double sOffsetToKeepTimeStart = currTime;
        double sOffsetToKeepTimeEnd = sOffsetToKeepTimeStart;
        double posToKeepStart = currPos;
        double posToKeepEnd = currPos;
        double targetPosToKeepEnd = currPos + uniformInterval * 2.0;
        double sOffset = currFTS.sOffset();

        while (posToKeepEnd < targetPosToKeepEnd && toKeep < state.remaining_path_x.size()) {
            toKeep++;
            sOffsetToKeepTimeEnd += currFTS.dt();
            posToKeepEnd = currFTS.s().position(sOffsetToKeepTimeEnd);
        }

        // number of time steps that have been consumed
        // number of time steps that remain that should be kept
        toKeep = (int) fmin(toKeep, state.remaining_path_x.size());

        // elapsed time to the new 'reference point', the origin of the new jmt for this update
        double deltaT = sOffsetToKeepTimeEnd;
        double posNewOrigin = currFTS.s().position(deltaT);

        double timeToLiveS = toKeep * nextFTS.dt();
        double timeToLiveD = currFTS.dCTS().timeToLive();

        //use the previous jmt and final reference point to determine where the
        std::vector<double> stateS = {
            currFTS.s().position(deltaT) - posNewOrigin,
            currFTS.s().speed(deltaT),
            currFTS.s().accel(deltaT)
        };
        std::vector<double> stateD = {
            currFTS.d().position(deltaT),
            currFTS.d().speed(deltaT),
            currFTS.d().accel(deltaT)
        };

        double currSpeed = currFTS.s().speed(currTime);

        //assume that we are at the initial state when there are no points
        if (remainingPoints == 0) {
            //toKeep = 50;
            //sOffsetToKeepTimeEnd += nextCTS.dt * toKeep;
            sOffset = state.s;
            stateD[0] = state.d;
            lastCollision.lane = (int) (state.d / laneWidth);
            nextFTS.d(JMT().init(stateD, stateD, nextFTS.dt()), timeToLiveS);
        }

        if (fabs(stateD[0] - state.d) > 3) {
            throw std::exception();
        }

        auto currentLane = (int) (stateD[0] / 4.0);

        // determine jmt using start and end state

        // amount of time necessary to generate a full prediction horizon
        double remT = dT - toKeep * currFTS.dt();
        if (remT <= toKeep * currFTS.dt()) {
            throw std::exception();
        }

        std::vector<JMT> bestCurvesS;
        //printf("curr predicted speed %f, state speed %f", currSpeed, state.speed);

        double furthest_s;
        double timePeriod = remT;
        double time = timePeriod;
        if (time > 0) {
            furthest_s = fmax(max_velocity * time, POSeval(stateS, time));
            bestCurvesS = nextFTS.sCTS().searchJMTSpace(stateS, furthest_s, max_velocity, {max_velocity, max_accel, max_jerk}, time, nextFTS.dt());
        }

        /*
        if (bestCurvesS.empty()) {
            throw std::exception();
        }
         */

        /*if (fabs(state.d - stateD[0]) > 1) {
            throw std::exception();
        }*/

        printf("toKeep %d, currPos %f, posToKeepStart %f, posToKeepEnd %f, sOffsetToKeepTimeStart %f, sOffsetToKeepTimeEnd %f, timeToLiveD %lf, sOffset %lf, dOffset %lf\n",
               toKeep, currPos, posToKeepStart, posToKeepEnd, sOffsetToKeepTimeStart, sOffsetToKeepTimeEnd, timeToLiveD, sOffset, stateD[0]);

        sOffset += posNewOrigin;
        if (sOffset > s_max_) {
            sOffset -= s_max_;
            printf("sOffset has been reset to  %f\n", sOffset);
        }

        nextFTS.s(bestCurvesS[0],
                  timeToLiveS,
                  sOffset,
                  (int) fmin(toKeep, remainingPoints));

        Vehicle ego;
        // collision uses the actual 's' because sOffset accrues error as the car turns along the road.
        FTS egoFTS = nextFTS;
        egoFTS.s(egoFTS.s(), timeToLiveS, state.s, 0);
        ego.initWithFTS(egoFTS, CAR_WIDTH, CAR_LENGTH);

        // create the vehicle states at the current time
        std::vector<Vehicle> other_vehicles = create_vehicles(state.sensor_state, COLLISION_HORIZON, 0);

        printf("Ego vehicle\n");
        printf("[id %d, pos %f, speed %f, lane %f]\n", ego.id(), ego.fts().s().position(0) + ego.fts().sOffset(), ego.fts().s().speed(0), ego.fts().d().position(0) /  laneWidth);
        printf("Vehicles\n");
        for (Vehicle v : other_vehicles) {
            printf("[id %d, pos %f, speed %f, lane %f], ", v.id(), v.fts().s().position(0), v.fts().s().speed(0), v.fts().d().position(0) /  laneWidth);
        }
        printf("\n");

        //check for collisions
        auto overlaps = check_overlap(other_vehicles, ego, remT);
        if (!overlaps.empty()) {

            //printf("overlaps detected, calculating new d trajectory\n");

            //determine the best lane

            Collision bestCollision = determine_best_lane_collision(ego, other_vehicles, currentLane, COLLISION_HORIZON);
            int newLane = bestCollision.lane;

            if (newLane == -1) {
                printf("Error: invalid lane.\n");
                throw std::exception();
            }

            if (bestCollision.id != -1) {

                const Vehicle& collidingVehicle = other_vehicles[bestCollision.id];
                double collisionS = bestCollision.position;//nextFTS.s().position(bestCollision.time);
                //do not speed up beyond the passing speed
                double distanceToVehicle = collidingVehicle.fts().s().position(0) + collidingVehicle.fts().sOffset() -
                                           (ego.fts().s().position(0) + ego.fts().sOffset());
                double collisionSpeed = fmin(fmin(bestCollision.speed, stateS[1] + PASSING_SPEED_INCREMENT), max_velocity);
                double collisionT = bestCollision.time;

                //only adjust speed when collision is within a distance threshold
                if (distanceToVehicle <= COLLISION_DISTANCE_THRESHOLD) {

                    /* Debugging */
                    if (currentLane != newLane) {
                        //determine new max speed
                        printf("new lane: lane %d, collision id %d, pos %f, time %f, speed %f\n",
                               newLane, bestCollision.id, collisionS, collisionT, collisionSpeed);
                    } else {
                        printf("collision detected: id %d, pos %f time %f, speed %f\n", bestCollision.id, collisionS,
                               bestCollision.time, collisionSpeed);
                    }

                    double dist = fmax(collisionS / 2.0, collisionS - MINIMUM_FOLLOW_DISTANCE);
                    if (dist < MINIMUM_FOLLOW_DISTANCE) {
                        //slow down
                        collisionSpeed -= 1.0;
                    }
                    collisionSpeed = fmax(1.0, collisionSpeed);
                    collisionT = fmax(1.0, collisionT);

                    //determine the state at time remT
                    std::vector<double> end = {
                            dist,
                            collisionSpeed,
                            0
                    };
                    JMT estimated = JMT().init(stateS, end, collisionT);
                    dist = estimated.position(remT);
                    std::vector<double> endEst = {
                            dist,
                            estimated.speed(remT),
                            estimated.accel(remT)
                    };
                    std::vector<double> stdDev = { dist, 2, 2 };

                    std::vector<double> limits = {max_velocity, max_accel, max_jerk};
                    //bestCurvesS = nextFTS.sCTS().searchJMTSpace(stateS, collisionS, collisionSpeed, {max_velocity, max_accel, max_jerk}, collisionT, nextFTS.dt());
                    bestCurvesS = nextFTS.sCTS().searchJMTs(stateS, endEst, stdDev, limits, 100, remT, nextFTS.dt(), 0.0);

                    /*
                    if (bestCurvesS.empty()) {
                        //
                        //bestCurvesS = nextFTS.sCTS().searchJMTs(stateS, end, stdDev, limits, 100, collisionT, nextFTS.dt(), 2, true);
                        throw std::exception();
                    }*/
                }
            }

            lastCollision.speed = stateS[1] + PASSING_SPEED_INCREMENT;
            if (lastCollision.lane != newLane) {
                printf("lane change initiating: lane %d\n", newLane);
                lastCollision.lane = newLane;
            }

            if (!bestCurvesS.empty())
                nextFTS.s(bestCurvesS[0],
                          timeToLiveS,
                          sOffset,
                          (int) fmin(toKeep, remainingPoints));

        }

        //Experimental
#if 0
        else if (timeToLiveD > 0.0001){

            //lane change is executing... to minimize normal acceleration increases, avoid accelerating tangentially while jerk is
            // positive (lateral acceleration is increasing)
            double latSpeed = currFTS.d().speed(timeToLiveS + currFTS.dCTS().deltaTime());
            double latAccel = currFTS.d().accel(timeToLiveS + currFTS.dCTS().deltaTime());
            if (fabs(latSpeed) > 0.1) {
                printf("reduce acceleration, lateral accel %f, accel %f, lastCollisionSpeed %f, latSpeed %f, speed %f\n", latAccel, stateS[2], lastCollision.speed, latSpeed, stateS[1]);
                double posSlow = lastCollision.speed * remT;
                nextFTS.setLimits(lastCollision.speed, max_accel / 2, max_jerk);

                std::vector<double> stdDev = { 5, 0.1, 1 };
                std::vector<double> end = {
                        posSlow,
                        lastCollision.speed,
                        0
                };
                bestCurvesS = nextFTS.sCTS().searchJMTs(stateS, end, stdDev, 100, remT, nextFTS.dt(), 0.4);
                if (bestCurvesS.empty()) {
                    throw std::exception();
                }
                nextFTS.s(bestCurvesS[0], timeToLiveS, sOffset, (int) fmin(toKeep, remainingPoints));
            }

        }
#endif

        if (state.forceLane != -1) {
            lastCollision.lane = state.forceLane;
        }

        //generate jmt 'd' trajectory
        if (currentLane != lastCollision.lane) {
            timeToLiveD = LANE_CHANGE_TIME;
            if (abs(currentLane - lastCollision.lane) > 1)
                timeToLiveD = LANE_CHANGE_TIME_EXTENDED;
        }

        switch_lanes(nextFTS, stateD, currentLane, lastCollision.lane, remT, timeToLiveD);


        return nextFTS;

    }

    /* Cost (or score) evaluation of various lane paths */

    Collision determine_best_lane_collision(Vehicle& ego, const std::vector<Vehicle>& other_vehicles, int currentLane, double dT) {

        const double COLLISION_PROXITY_WEIGHT = 150;
        const double LANE_GAP_WEIGHT = 75;
        const double LANE_GAP_THRESHOLD = 10.0;
        const double VEHICLE_SPEED_WEIGHT = 25.0;
        const double KEEP_LANE_WEIGHT = 10.0;
        const double ADJACENT_LANE_WEIGHT = 2.0;
        const double LEAST_TRAFFIC_LANE_WEIGHT = 15.0;
        const double CURRENT_LANE_COLLISION_THRESHOLD_TIME = 1.5;
        const double CURRENT_LANE_COLLISION_WEIGHT = 200.0;
        const double MIDDLE_LANE_COLLISION_THRESHOLD_TIME = 4.0;
        const double MIDDLE_LANE_COLLISION_PENALTY_WEIGHT= -200.0;
        const double COLLISION_LIKELIHOOD_THRESHOLD_TIME = 1.5;
        const double COLLISION_LIKELIHOOD_THRESHOLD_DISTANCE = CAR_LENGTH * 1.5;
        const double ACTIVE_LANE_CHANGE_WEIGHT = 0;//150.0;
        const double LANE_CHANGE_SPEED_PENALTY_WEIGHT = -100.0;
        const double LANE_CHANGE_SPEED_THRESHOLD = 40.0;
        const double STUCK_IN_LANE_THRESHOLD_TIME_LIMIT = 15.0;
        const double STUCK_IN_LANE_THRESHOLD_DISTANCE = MINIMUM_FOLLOW_DISTANCE * 4;

        //determine collisions in each lane
        std::vector<double> costs(MAX_LANES);
        std::vector<Collision> collision_lanes(MAX_LANES);
        std::vector<int> car_count_lanes(MAX_LANES);

        double currPos = ego.fts().s().position(0) + ego.fts().sOffset();
        double finalPos = ego.fts().s().position(dT) + ego.fts().sOffset();
        double currSpeed = ego.fts().s().speed(0);

        int newLane = currentLane;
        Vehicle egoCopy = ego;
        for (int i = 0; i < MAX_LANES; i++) {
            std::vector<double> otherLaneState = {i * laneWidth + laneWidth / 2, 0, 0};
            JMT dOtherLane = JMT().init(otherLaneState, otherLaneState, dT);

            FTS hypoLaneT = egoCopy.fts();
            hypoLaneT.d(dOtherLane, dT);
            egoCopy.initWithFTS(hypoLaneT, CAR_WIDTH, CAR_LENGTH);
            auto collisions = check_collisions(other_vehicles, egoCopy, dT);

            //filter collisions - eliminate collisions with low likelihood.
            auto it = collisions.begin();
            while (it != collisions.end()) {
                Collision& c = *it;
                if (c.time != -1 && c.lane != currentLane) {
                    Vehicle v = other_vehicles[c.id];
                    double vehiclePosNow = v.fts().s().position(0) + v.fts().sOffset();
                    double posDelta = egoCopy.fts().s().position(COLLISION_LIKELIHOOD_THRESHOLD_TIME) + egoCopy.fts().sOffset()
                                      - (v.fts().s().position(COLLISION_LIKELIHOOD_THRESHOLD_TIME) + v.fts().sOffset());
                    //vehicle must be behind us to have low probability of collision
                    if (currPos > vehiclePosNow + v.length() &&
                        posDelta > COLLISION_LIKELIHOOD_THRESHOLD_DISTANCE) {
                        //ignore this collision
                        printf("collision with id %d at time %f ignored (delta %f)\n", c.id, c.time, posDelta);
                        it = collisions.erase(it);
                        continue;
                    }
                }
                it++;
            }

            auto collision = find_nearest_collision(collisions);

            collision_lanes[i] = collision;
            collision_lanes[i].lane = i;
        }

        //prefer the current lane
        costs[currentLane] += 1 * KEEP_LANE_WEIGHT;

        if (ego.fts().dCTS().timeToLive() > 0.0001) {
            //prefer the active lane change
            costs[lastCollision.lane] += 1 * ACTIVE_LANE_CHANGE_WEIGHT;
        }

        //prefer adjacent lanes to the current lane
        if (currentLane == 1) {
            costs[0] += 1 * ADJACENT_LANE_WEIGHT;
            costs[2] += 1 * ADJACENT_LANE_WEIGHT;
        } else {
            costs[1] += 1 * ADJACENT_LANE_WEIGHT;
        }

        // score based on number of vehicles ahead (within the horizon)
        for (Vehicle v: other_vehicles) {
            int lane = (int)(v.fts().d().position(0) / laneWidth);
            double distance = v.fts().s().position(0);
            if (distance >= currPos && distance <= finalPos)
                car_count_lanes[lane]++;
        }

        for (int i = 0; i < MAX_LANES; i++) {
            costs[i] += LEAST_TRAFFIC_LANE_WEIGHT / (car_count_lanes[i] + 1);
        }

        //is there a potential collision by changing between two lanes?
        if ((currentLane == 0 || currentLane == 2) && collision_lanes[1].time < MIDDLE_LANE_COLLISION_THRESHOLD_TIME) {
            costs[currentLane == 0 ? 2 : 0] += MIDDLE_LANE_COLLISION_PENALTY_WEIGHT;
        }

        //would the max accel be violated if a lane change was initiated?
        if (currSpeed < LANE_CHANGE_SPEED_THRESHOLD && (currentLane == 0 || currentLane == 2)) {
            costs[currentLane == 0 ? 2 : 0] -= LANE_CHANGE_SPEED_THRESHOLD;
        }

        // if threre's a collision too close in the current lane, then current lane will have very high weight
        double currentLaneCollisionWeight = CURRENT_LANE_COLLISION_THRESHOLD_TIME - collision_lanes[currentLane].time;
        if (currentLaneCollisionWeight > 0) {
            costs[currentLane] += currentLaneCollisionWeight * CURRENT_LANE_COLLISION_WEIGHT;
        }

        printf("   ttl d %f\n", ego.fts().dCTS().timeToLive());

        if (collision_lanes[currentLane].id != -1 &&
            collision_lanes[currentLane].position < STUCK_IN_LANE_THRESHOLD_DISTANCE &&
            ego.fts().dCTS().timeToLive() < -STUCK_IN_LANE_THRESHOLD_TIME_LIMIT) {
            printf("Stuck in lane for too long... slow down... to try and find another lane\n");
            collision_lanes[currentLane].speed = fmax(collision_lanes[currentLane].speed -10.0, 0);
        }

        printf("Lane costs based on biases\n");
        for (int i = 0; i < MAX_LANES; i++) {
            printf("[lane %d: %f], ", i, costs[i]);
        }
        printf("\n");

        // score based on furthest collision proximity to ego vehicle
        printf("Lane costs based on proximity\n");
        for (int i = 0; i < MAX_LANES; i++) {
            costs[i] += (collision_lanes[i].time != -1 ? (collision_lanes[i].time / dT) : 1) * COLLISION_PROXITY_WEIGHT;
            printf("[lane %d: %f], ", i, costs[i]);
        }
        printf("\n");

        // compare lanes to each other
        printf("Lane costs based on relative characteristics\n");
        for (int i = 0; i < MAX_LANES; i++) {
            Collision collision = collision_lanes[i];
            Vehicle vehicleInLane;
            if (collision.id != -1)
                vehicleInLane = other_vehicles[collision.id];
            std::vector<int> other_lane = {1};
            if (i == 1)
                other_lane = {0,2};
            for (int lane : other_lane) {
                Collision otherCollision = collision_lanes[lane];
                Vehicle vehicleInOtherLane;
                if (otherCollision.id != -1)
                    vehicleInOtherLane = other_vehicles[otherCollision.id];

                // score based on gap between lanes
                if (collision.time != -1 && otherCollision.time != -1) {
                    //find the positions of the vehicles at the greatest time
                    double time = collision_lanes[i].time;
                    double posInLane = vehicleInLane.fts().s().position(time);
                    double posInOtherLane = vehicleInOtherLane.fts().s().position(time);

                    //this is a binary result, it either allows you to switch lanes well enough or not
                    if (posInLane - posInOtherLane >= LANE_GAP_THRESHOLD) {
                        // consider which side the lane gap is on... higher weight if it's nearer to current lane
                        costs[i] += LANE_GAP_WEIGHT * (abs(lane - currentLane) <= 1 ? 1.0 : 0.25);
                        printf("    lane gap found for lane %d and %d, between %d (%f time %f pos) and %d (%f pos)\n",
                               i, lane, vehicleInLane.id(), time, posInLane, vehicleInOtherLane.id(), posInOtherLane);
                    }
                } else if (collision.time == -1) {
                    costs[i] += LANE_GAP_WEIGHT;
                    printf("    lane gap found for lane %d due to no collisions\n", i);
                }

                // score based on relative differences in vehicles speeds
                if (collision.time != -1 && otherCollision.time != -1) {
                    double delta_speed = vehicleInLane.fts().s().speed(collision.time) - vehicleInOtherLane.fts().s().speed(collision.time);
                    if (delta_speed > 0) {
                        costs[i] += delta_speed / MAX_RELATIVE_VELOCITY * VEHICLE_SPEED_WEIGHT;
                    }
                } else if (collision.time == -1) {
                    costs[i] += VEHICLE_SPEED_WEIGHT;
                }
            }

            printf("[lane %d: %f], ", i, costs[i]);
        }
        printf("\n");

        /*if (ego.fts().s().speed(0) > 17.f) {
            costs[currentLane] += 400;
        }*/

        //choose the lane with the highest score
        return collision_lanes[std::distance(costs.begin(), std::max_element(costs.begin(), costs.end()))];
    }

    void switch_lanes(FTS& nextFTS, std::vector<double> start, int currentLane, int newLane, double dT, double timeToLiveD) {
        std::vector<JMT> bestCurvesD;

        //generate jmt 'd' trajectory
        //position the car barely in the lane
        double currPos = nextFTS.s().position(0) + nextFTS.sOffset();
        //XXX fix error in waypoints that might cause car to drive barely off the road.
        double deltaF = currPos > 4500 && currPos < 6000 && newLane == 2 ? - (laneWidth - CAR_WIDTH) / 2 : 0;
        double endD = (newLane * laneWidth + laneWidth / 2) + deltaF;

        printf("calculating lane position: lane %d, startD %f, endD %f\n", newLane, start[0], endD);

        CTS dCTS = CTS();
        std::vector<double> end = {endD, 0, 0};
        std::vector<double> stdDev = {0, 0, 0};
        std::vector<double> limits = {D_MAX_VELOCITY, D_MAX_ACCEL_JERK, D_MAX_ACCEL_JERK};
        bestCurvesD = nextFTS.dCTS().searchJMTSpace(start, endD, 0, limits, dT, nextFTS.dt());

        /*
        if (bestCurvesD.empty()) {
            throw std::exception();
        }

        if (fabs(bestCurvesD[0].position(0) - bestCurvesD[0].position(dt)) > 0.1) {
            throw std::exception();
        }
        */

        if (!bestCurvesD.empty()) {
            printf("updating D trajectory: ttl %f, d %f\n", timeToLiveD, bestCurvesD[0].position(0));
            nextFTS.d(bestCurvesD[0],
                      timeToLiveD);
        }

        if (newLane != currentLane)
            printf("changing lanes: lane %d change ttl %f", newLane, timeToLiveD);
    }

    std::vector<Vector2d> generate_path_points(SplineCommonType* spline, Waypoints& waypoints,
                                               const FTS& fts, std::vector<Vector2d> points,
                                               int startI, double tOffset,
                                               double s_start, double s_end, double s_max,
                                               double d_start, bool useD,
                                               int maxItems, bool validate) {

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
            s_diff = s_max - waypoints.s[prevWaypoint];
            s_diff += waypoints.s[nextWaypoint];
        }

        if (waypointDiff <= 0) {
            waypointDiff += waypoints.s.size();
        }

        //printf("prevWaypoint %d, nextWaypoint %d\n", prevWaypoint, nextWaypoint);

        double maxAccelN = 0;
        std::vector<double> sv;
        int j = 0, i = startI;
        double s_rate = s_diff / waypointDiff;
        double dRemT = fts.dCTS().timeToLive();
        for (j = 0; i < maxItems; i++, j++) {
            double s_delta_eval = fts.s().position(fts.sCTS().deltaTime() + j * fts.dt() + tOffset) + s_start;
            //if (s_delta_eval > s_end + 0.0001) {
            //    break; //end early
            //}
            double s_curr_delta = s_delta_eval - waypoints.s[prevWaypoint];
            double s = (s_curr_delta / s_rate) + prevWaypoint;
            SplineCommonType::InterpolatedPTC curvature = spline->getCurvature(s);
            Eigen::Vector2d normal = getNormalFromCurvature(curvature, s);

            double x = curvature.position[0];
            double y = curvature.position[1];

            if (useD) {
                double d = fts.d().position(fts.dCTS().deltaTime() + j * fts.dt()) + d_start;
                //printf("[j %d, offset %f, d %f], ", j, fts.dCTS().deltaTime() + j * fts.dt() + tOffset, d);

                x += normal[0] * d;
                y += normal[1] * d;
            }

            Vector2d point;

            point[0] = x;
            point[1] = y;

            if (validate) {
                if (!points.empty()) {
                    Vector2d &prevPoint = points[points.size() - 1];
                    double dist = distance(point[0], point[1], prevPoint[0], prevPoint[1]);
                    double dists = 0;
                    if (sv.size() > 0) {
                        dists = s_curr_delta - sv[sv.size() - 1];
                        if (validate && dists > SPEED_LIMIT_VALIDATION * 0.02f) {
                            throw std::exception();
                        }
                    }
                    if (validate && dist > SPEED_LIMIT_VALIDATION * 0.02f) {
                        throw std::exception();
                    }
                }

                if (points.size() > 1) {
                    Vector2d &point1 = points[points.size() - 1];
                    Vector2d &point2 = points[points.size() - 2];
                    Vector2d v2 = point1 - point2;
                    Vector2d v1 = point - point1;
                    Vector2d a = v2 - v1;
                    double dist1 = distance(point1[0], point1[1], point[0], point[1]);
                    double dist2 = distance(point1[0], point1[1], point2[0], point2[1]);

                    double speed = fts.s().speed(fts.sCTS().deltaTime() + j * fts.dt() + tOffset);
                    double k = getCurvatureScalerFromCurvature(curvature);
                    double accelN = k * speed * speed;

                    if (validate && accelN > MAX_ACCEL_JERK) {
                        //throw std::exception();
                    }
                    if (v1.dotProduct(v1, v2) < 0) {
                        //point is not continuous
                        throw std::exception();
                    }

                    if (accelN > maxAccelN) maxAccelN = accelN;
                }
            }

            points.push_back(point);
            sv.push_back(s_curr_delta);
        }
        //printf("\n");

        if (validate) {
            printf("maximum normal acceleration %f", maxAccelN);
        }

        return points;
    }

    std::vector<double> getFrenet(double s, double x, double y, const Waypoints& waypoints, SplineCommonType& spline, double interval, bool validate = false) {

        int i = (int) s, next_wp, prev_wp, size = waypoints.s.size();
        double dist = !waypoints.s.empty() ? waypoints.s[waypoints.s.size() - 1] : 0, prevDistance = 0;
        int closest = -1, end = i + 10;
        i -= fmax(waypoints.s.size(), 10); //get us there a little sooner than iterating the whole list
        if (i < 0) i += size;
        if (end >= size) end -= size;
        while (i != end) {
            double nextDistance = distance(waypoints.x[i], waypoints.y[i], x, y);
            if (nextDistance < dist) {
                closest = i;
                dist = nextDistance;
            }
            i++;
            i >= size ? i = 0 : i;
        }

        if (validate && dist > interval + 0.01) {
            //the distance should not be this large.
            throw std::exception();
        }

        int next = (closest + 1) % size;
        int prev = (closest - 1); if (prev < 0) prev = size;

        if (distance(waypoints.x[next], waypoints.y[next], x, y) <
            distance(waypoints.x[prev], waypoints.y[prev], x, y)) {
            next_wp = next;
            prev_wp = closest;
        } else {
            next_wp = closest;
            prev_wp = prev;
        }

        double n_x = waypoints.x[next_wp]-waypoints.x[prev_wp];
        double n_y = waypoints.y[next_wp]-waypoints.y[prev_wp];
        double x_x = x - waypoints.x[prev_wp];
        double x_y = y - waypoints.y[prev_wp];

        // find the projection of x onto n
        double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        double proj_x = proj_norm*n_x;
        double proj_y = proj_norm*n_y;

        double frenet_d = distance(x_x,x_y,proj_x,proj_y);

        //see if d value is positive or negative by comparing it to a center point

        double center_x = 1000-waypoints.x[prev_wp];
        double center_y = 2000-waypoints.y[prev_wp];
        double centerToPos = distance(center_x,center_y,x_x,x_y);
        double centerToRef = distance(center_x,center_y,proj_x,proj_y);

        if(centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }

        // calculate s value
        double frenet_s = waypoints.s[prev_wp];

        frenet_s += distance(0,0,proj_x,proj_y);

        /* There's error in the projection usually, to be as precise as possible, a search must be conducted to find as best of a match */
        double length = 0;
        double dt = 0.0, factor = 1.f;
        bool incu = false, incd = false;
        double precision = 0.000001;
        double product = 0;
        do {
            //generate a new set of waypoints that are uniformly separated by arc-length
            SplineCommonType::InterpolatedPTC curvature = spline.getCurvature((frenet_s + dt)/ interval);
            Vector2d pos = curvature.position;
            Vector2d tangent = curvature.tangent;
            Vector2d normal;

            normal[0] = x - pos[0];
            normal[1] = y - pos[1];

            product = tangent.dotProduct(normal, tangent);

            if (product > precision) {
                dt += factor;
                incu = true;
            } else if (product < -precision) {
                dt -= factor;
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

        } while (fabs(product) > precision);

        frenet_s += dt;

        return {frenet_s,frenet_d};

    }

    std::vector<Vehicle> create_vehicles(const std::vector<SensorVehicleState>& sensorVehicleStates, double dT, double timeOffset) {
        std::vector<Vehicle> retval;
        for (auto& s : sensorVehicleStates) {
            Vehicle v;
            v.initWithSensorVehicleState(s, CAR_WIDTH, CAR_LENGTH, dT, timeOffset);
            retval.push_back(v);
        }
        return retval;
    }

    std::vector<Vehicle> check_overlap(const std::vector<Vehicle>& vehicles, const Vehicle& ego, double dT) {
        std::vector<Vehicle> retval;
        for (const Vehicle& s : vehicles) {
            if (s.does_overlap(ego,dT,laneWidth)) {
                retval.push_back(s);
            }
        }
        return retval;
    }

    std::vector<Collision> check_collisions(const std::vector<Vehicle>& vehicles,
                                                const Vehicle& ego,
                                                double dT) {
        printf("checking collisions with ego vehicles at pos %f, lane %f\n",
               ego.fts().s().position(0) + ego.fts().sOffset(),
               ego.fts().d().position(0) / laneWidth
        );
        std::vector<Collision> retval;
        for (const Vehicle& vehicle : vehicles) {
            bool doesCollide = false;
            //determine exact moment of collision
            double collisionT = vehicle.collisionTime(ego, dT, COLLISION_BUFFER_RANGE);
            int lane = (int) (vehicle.fts().d().position(collisionT) / laneWidth);
            if (collisionT != -1) {
                printf("    collision detected with %d at pos %f, time %f, lane %f, speed %f (ego lane %f)\n",
                       vehicle.id(), vehicle.fts().s().position(collisionT), collisionT,
                       vehicle.fts().d().position(collisionT) / laneWidth,
                       vehicle.fts().s().speed(collisionT),
                       ego.fts().d().position(collisionT) / laneWidth
                );
                retval.push_back({vehicle.id(), lane, collisionT, ego.fts().s().position(collisionT), vehicle.fts().s().speed(collisionT) });
            }
        }
        return retval;
    }

    Collision find_nearest_collision(const std::vector<Collision>& collision) {
        Collision retval = { -1, -1, -1, -1, -1 };
        for (const Collision& c : collision) {
            if (retval.id == -1 || c.time < retval.time) {
                retval = c;
            }
        }
        return retval;
    }
};

PathGenerator::~PathGenerator() = default;

PathGenerator::PathGenerator(Waypoints waypoints, double s_max) :
    pimpl(new PathGenerator::impl(waypoints, s_max)) {
}

PathPoints PathGenerator::generate_path(VehicleState state) {
    return pimpl->generate_path(state);
}
