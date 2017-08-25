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

static const int MAX_LANES = 3;

static Eigen::Rotation2D<double> g_rotNormal(-90.f * M_PI / 180.f);

extern double distance(double x1, double y1, double x2, double y2);

double SPEEDeval(const std::vector<double>& params, double T) {
    double t2 = T;
    return params[1] + 2 * params[2] * t2;
}

double POSeval(const std::vector<double>& params, double T) {
    double t2 = T * T;
    double val = params[0] + params[1] * T + params[2] * t2;
    return val;
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

    printf("Total error = %f\n", err);

    return newWaypoints;
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

class JMT {

private:

    std::vector<double> params;
    double Tmax;
    double cost;

public:

    JMT() : params(std::vector<double>(6)) {
        cost = 0;
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
        this->cost = 0;

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

};

/* Composite Trajectory Segment */
class CTS {

private:

    JMT jmt;
    double timeToNextJMT;
    double timeOffsetJMT;

    double max_velocity;
    double max_accel;
    double max_jerk;

public:
    CTS() {
        timeToNextJMT = 0;
        timeOffsetJMT = 0;
        max_velocity = 0;
        max_jerk = 0;
        max_accel = 0;
    }

    CTS& init(double max_velocity, double max_accel, double max_jerk) {
        this->max_velocity = max_velocity;
        this->max_accel = max_accel;
        this->max_jerk = max_jerk;
        return *this;
    }

    bool validateJMT(const JMT& jmtCurve, double dT, double dt) const {
        //does it violate acceleration
        bool failedConstraint = false;
        for (double t = 0; t < dT && !failedConstraint; t += dt) {
            double accel = jmtCurve.accel(t);
            if (fabs(accel) > max_accel) {
                failedConstraint = true;
                //printf("accel violation: %f\n", accel);
                break;
            }
        }

        //does it violate speed
        for (double t = 0; t < dT && !failedConstraint; t += dt) {
            double speed = jmtCurve.speed(t);
            if (fabs(speed) > max_velocity) {
                failedConstraint = true;
                //printf("speed violation: %f\n", speed);
                break;
            }
        }

        //does it violate jerk
        for (double t = 0; t < dT && !failedConstraint; t += dt) {
            double jerk = jmtCurve.jerk(t);
            if (fabs(jerk) > max_jerk) {
                failedConstraint = true;
                //printf("jerk violation: %f\n", jerk);
                break;
            }
        }

        return !failedConstraint;
    }

    std::vector<JMT> searchJMTSpace(double pos, double speed, double accel, double final_s, double final_velocity, double dT, double dt, bool bPermuteS = true) const {
        JMT bestCurve;

        permuteParameter(0.0, dt * 10, dt, true, true,
            [this, &bestCurve, &final_s, &pos, &speed, &accel, &dT, &dt, &final_velocity, &bPermuteS](double s_ddot)->bool {

                double s_dot_inc = fabs(fmin(dt * 10, (final_velocity - speed) / 10));

                return permuteParameter(final_velocity, final_velocity - speed, s_dot_inc, false, true,
                    [this, &bestCurve, &final_s, &pos, &speed, &accel, &s_ddot, &dT, &dt, &bPermuteS](double s_dot)->bool {

                     return permuteParameter(final_s, final_s - pos, dt, bPermuteS, bPermuteS,
                        [this, &bestCurve, &pos, &speed, &accel, &s_dot, &s_ddot, &dT, &dt](double s)->bool {

                        std::vector<double> start{pos, speed, accel};
                        std::vector<double> end{s, s_dot, s_ddot};

                        if (speed > 21) {
                            printf("jmt attempt (pos %f, speed %f, accel %f)\n", s, s_dot, s_ddot);
                        }

                        if (s < 0) {
                            throw std::exception();
                        }

                        JMT params = JMT().init(start, end, dT);

                        if (this->validateJMT(params, dT, dt)) {
                            bestCurve = params;
                            printf("jmt final (pos %f, speed %f, accel %f)\n", s, s_dot, s_ddot);
                            return true;
                        }

                        return false;
                    });

                });

            });

        if (bestCurve.isDefined())
            return {bestCurve};

        return {};
    }

    void defineJMT(const JMT& jmt, double timeToLive) {
        this->jmt = jmt;
        this->timeToNextJMT = timeToLive;
        this->timeOffsetJMT = 0;
    }

    void updateTime(double dT) {
        timeToNextJMT -= dT;
        timeOffsetJMT += dT;
    }

    double currentTime() const { return - timeToNextJMT; }

    double timeToLive() const { return timeToNextJMT; }

    double offsetTime() const { return timeOffsetJMT; }

    operator const JMT&() const { return jmt; }

};

/* Frenet Trajectory Segment */
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
        _dCTS.init(max_velocity, max_accel, max_jerk);
        _sCTS.init(max_velocity, max_accel, max_jerk);
        _dt = dt;
    }

    const JMT& d() const { return _dCTS; }
    const JMT& s() const { return _sCTS; }
    const CTS& sCTS() const { return _sCTS; }
    const CTS& dCTS() const { return _dCTS; }
    double dt() const { return _dt; }
    double sOffset() const { return _sOffset; }
    int toKeep() const { return _toKeep; }

    void updateTime(double dT) {
        _dCTS.updateTime(dT);
        _sCTS.updateTime(dT);
    }

    void updateJMTs(const JMT& s, const JMT& d, double sTTL, double dTTL, double sOffset, int toKeep) {
        _sCTS.defineJMT(s, sTTL);
        _dCTS.defineJMT(d, dTTL);
        _toKeep = toKeep;
        _sOffset = sOffset;
    }

};

typedef struct {
    int id;
    int lane;
    double start;
    double end;
    double speed;
} CollisionBoundingBox;

typedef std::vector<CollisionBoundingBox> VecCollisionBoundingBox;

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

double calculateCollisionMoment(SensorVehicleState state, const JMT& jmt, double d, double dT, double dt, double laneWidth, double range) {
    double t = 0;
    double start = jmt.position(0);
    double s = 0;
    while (t <= dT) {
        s = jmt.position(t);
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

class PathGenerator::impl {
public:

    impl();
    ~impl();
    int find_waypoint_floor(std::vector<double>& waypoints, double s);
    double find_target_t(std::vector<double>& waypoints, double s);
    double find_closest_time(double s);
    FTS updateTrajectory(const FTS& currFTS, VehicleState state, double max_velocity, double max_accel, double max_jerk) {

        FTS nextFTS = currFTS;
        nextFTS.init(max_velocity, max_accel, max_jerk, dt);

        double currTime = currFTS.sCTS().currentTime();
        double currPos = currFTS.s().position(currTime);

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

        //assume that we are at the initial state when there are no points
        if (remainingPoints == 0) {
            //toKeep = 50;
            //sOffsetToKeepTimeEnd += nextCTS.dt * toKeep;
            sOffset = state.s;
        }

        // elapsed time to the new 'reference point', the origin of the new jmt for this update
        double deltaT = sOffsetToKeepTimeEnd;
        double posNewOrigin = currFTS.s().position(deltaT);

        //use the previous jmt and final reference point to determine where the
        double pos = currFTS.s().position(deltaT) - posNewOrigin; //subtract the waypoint origin
        double speed = currFTS.s().speed(deltaT);
        double accel = currFTS.s().accel(deltaT);
        double currSpeed = currFTS.s().speed(currTime);

        // determine jmt using start and end state

        // amount of time necessary to generate a full prediction horizon
        double remT = dT - toKeep * currFTS.dt();
        if (remT <= toKeep * currFTS.dt()) {
            throw std::exception();
        }

        std::vector<JMT> bestCurvesS;
        printf("curr predicted speed %f, state speed %f", currSpeed, state.speed);

        double furthest_s;
        double timePeriod = remT;
        double time = timePeriod;
        if (time > 0) {
            furthest_s = fmax(max_velocity * time, POSeval({pos, speed, accel}, time));
            bestCurvesS = nextFTS.sCTS().searchJMTSpace(pos, speed, accel, furthest_s, max_velocity, time, nextFTS.dt());
        }

        if (bestCurvesS.empty()) {
            throw std::exception();
        }

        /*
        //check for collisions
        auto overlaps = check_overlap(state.s, state.d, state.sensor_state, bestCurveS, remT, dt);
        if (!overlaps.empty() && nextCTS.dTimeToNextJMT <= 0) {

            //find closest colliding object
            auto collisions = check_collisions(state.sensor_state, overlaps, state.d, bestCurveS, remT, dt);

            if (!collisions.empty()) {
                CollisionBoundingBox collision;
                if (!collisions.empty())
                    collision = collisions.front();

                if (collision.id != -1) {

                    //determine if there's a free adjacent lane
                    auto overlapsLeft = check_overlap(state.s, fmax(0, state.d - 4.0), state.sensor_state, bestCurveS, remT, dt);
                    auto overlapsRight = check_overlap(state.s, fmin(10, state.d + 4.0), state.sensor_state, bestCurveS, remT, dt);
                    int currentLane = (int) (state.d / 4.0);
                    int newLane = currentLane;

                    if (overlapsLeft.empty()) {
                        newLane = (int) fmax(0, state.d - 4.0) / 4;
                    } else if (overlapsRight.empty()) {
                        newLane = (int) fmin(10.0, state.d + 4.0) / 4;
                    }

                    if (currentLane != newLane) {
                        //generate jmt 'd' trajectory
                        double startD = 0; //state.d - (currentLane * 4.0 + 2.0);
                        double endD = (newLane * 4.0 + 2.0) - (currentLane * 4.0 + 2.0);
                        auto bestCurveD = nextCTS.permuteJMT(startD - endD, 0, 0, 0, 0, remT, false);
                        if (!bestCurveD.isDefined()) {
                            throw new std::exception();
                        }

                        printf("changing lanes: lane %d, startD %f, endD %f\n", newLane, startD, endD);

                        nextCTS.dJMT = bestCurveD;
                        nextCTS.dTimeOffsetJMT = 0;
                        nextCTS.dTimeToNextJMT = remT;
                        nextCTS.currentLane = newLane;

                        //determine new max speed
                        bestCurveS = nextCTS.permuteJMT(pos, speed, accel, collision.start, max_velocity, remT);
                        collisionState = collision;
                        if (!bestCurveS.isDefined()) {
                            throw new std::exception();
                        }

                    } else {
                        printf("collision detected: id %d, speed %f pos %f\n", collision.id, collision.speed,
                               collision.start);
                        //determine new max speed
                        bestCurveS = nextCTS.permuteJMT(pos, speed, accel, fmax(collision.start - minimumFollowDistance, pos), collision.speed, remT);
                        collisionState = collision;
                        if (!bestCurveS.isDefined()) {
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
                bestCurveS = nextCTS.permuteJMT(pos, speed, accel, fmax(furthest_s - minimumFollowDistance, pos), collisionState.speed, remT);
            } else {
                //invalidate
                collisionState.id = -1;
            }

            if (!bestCurveS.isDefined()) {
                throw new std::exception();
            }
        } else {
            collisionState.id = -1;
        }
         */

        printf("toKeep %d, currPos %f, posToKeepStart %f, posToKeepEnd %f, sOffsetToKeepTimeStart %f, sOffsetToKeepTimeEnd %f\n",
               toKeep, currPos, posToKeepStart, posToKeepEnd, sOffsetToKeepTimeStart, sOffsetToKeepTimeEnd);

        nextFTS.updateJMTs(bestCurvesS[0],
                           currFTS.d(),
                           toKeep * nextFTS.dt(),
                           0,
                           sOffset + posNewOrigin,
                           (int) fmin(toKeep, remainingPoints));

        return nextFTS;

    }

    VecCollisionBoundingBox check_overlap(double s, double d, const std::vector<SensorVehicleState>& sensor_state, const JMT& jmt, double dT, double dt);
    VecCollisionBoundingBox check_collisions(const std::vector<SensorVehicleState>& vehicles,
                                             const VecCollisionBoundingBox& boxes,
                                             double d,
                                             const JMT& params,
                                             double dT,
                                             double dt);
    std::vector<Vector2d> generate_path_points(SplineType* spline, Waypoints& waypoints,
                                               const FTS& cts, std::vector<Vector2d> points,
                                               int startI, double tOffset, double s_start, double s_end, double s_max, double d_start,
                                               int maxItems, bool validate);

    double s_max_;
    Waypoints waypoints_[MAX_LANES];
    SplineType* spline_[MAX_LANES];
    FTS trajectory;
    double dT;
    double dt;
    double uniformInterval;
    double minimumFollowDistance;
    std::vector<Vector2d> prevPoints;
    CollisionBoundingBox collisionState;

    std::vector<double> getFrenet(VehicleState state, double x, double y, const Waypoints& waypoints, SplineType& spline, double interval) {

        int i = 0, next_wp, prev_wp;
        double dist = waypoints.s.size() > 0 ? waypoints.s[waypoints.s.size() - 1] : 0, prevDistance = 0;
        while (i < waypoints.s.size()) {
            double nextDistance = distance(waypoints.x[i], waypoints.y[i], x, y);
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
        frenet_s -= interval;

        /* There's error in the projection usually, to be as precise as possible, a search must be conducted to find as best of a match */
        double length = 0;
        double dt = 0.0, factor = 1.f;
        bool incu = false, incd = false;
        double precision = 0.000001;
        double product = 0;
        do {
            //generate a new set of waypoints that are uniformly separated by arc-length
            SplineType::InterpolatedPTC curvature = spline.getCurvature((frenet_s + dt)/ interval);
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

};

PathGenerator::PathGenerator() : pimpl(new impl()) {

}

PathGenerator::~PathGenerator() = default;

PathGenerator::PathGenerator(Waypoints waypoints, double s_max) : PathGenerator() {

    impl& im = *pimpl;
    im.s_max_ = s_max;
    im.uniformInterval = 1.f;
    im.minimumFollowDistance = 20.f;

    for (int i = 0; i < MAX_LANES; i++) {

        std::vector<Vector2d> xv_lane;
        for (int p = 0; p < waypoints.x.size(); p++) {
            Vector2d v;
            v[0] = waypoints.x[p];// + waypoints.dx[p] * (i * 4.0 + 2.0);
            v[1] = waypoints.y[p];// + waypoints.dy[p] * (i * 4.0 + 2.0);
            xv_lane.push_back(v);
        }

        Waypoints newWaypoints = generate_scaled_waypoints_from_points(xv_lane, 0, s_max, im.uniformInterval,
                                                                       im.uniformInterval);
        xv_lane = xyVectorsToVector2d(newWaypoints.x, newWaypoints.y);
        im.spline_[i] = new SplineType(xv_lane);
        im.waypoints_[i] = newWaypoints;
    }

    im.dT = 4.f;
    im.dt = 0.02;
}


PathPoints PathGenerator::generate_path(VehicleState state) {

    impl& im = *pimpl;

    int used = (int) fmax(0, (int) im.prevPoints.size() - (int) state.remaining_path_x.size());
    double lengthDistTraveled = arcLength(im.prevPoints, 0, used);

    //reduce time
    im.trajectory.updateTime(used * im.trajectory.dt());

    double offsetTime = im.trajectory.sCTS().offsetTime();

    printf("used %d, time delta %f, jmt time offset %f, jmt time remaining %f, distance traveled %f, speed %f, accel %f\n",
           used,
           used * im.trajectory.dt(),
           offsetTime,
           im.trajectory.sCTS().timeToLive(),
           lengthDistTraveled,
           im.trajectory.s().speed(offsetTime),
           im.trajectory.s().accel(offsetTime)
    );

    bool bSkip = im.trajectory.sCTS().timeToLive() > 0.0001;

    // add previous points
    PathPoints retval;
    std::vector<Vector2d> points;
    int i = 0;
    for (; i < state.remaining_path_x.size() && bSkip; i++) {
        Vector2d point;
        point[0] = state.remaining_path_x[i];
        point[1] = state.remaining_path_y[i];
        points.push_back(point);
        retval.x.push_back(point[0]);
        retval.y.push_back(point[1]);
    }

    if (bSkip) {
        im.prevPoints = points;
        printf("skipped - time to cover %f\n", im.trajectory.sCTS().timeToLive());
        return retval;
    }

    //generate points along this path in one second
    auto jmtParams = im.updateTrajectory(im.trajectory, state, 22.3, 10, 10);

    // add points to keep
    i = 0;
    for (; i < jmtParams.toKeep(); i++) {
        Vector2d point;
        point[0] = state.remaining_path_x[i];
        point[1] = state.remaining_path_y[i];
        points.push_back(point);
    }

    int currentLane = 0; //(int) round(jmtParams.currentLane);
    int targetItems = (int) (im.dT / im.dt);

    //localize the s/d based on the last point to keep
    double pos = state.s;
    if (im.prevPoints.size() > 0) {
        double deltaT = -im.trajectory.sCTS().currentTime();
        double currPos = im.trajectory.s().position(deltaT) + im.trajectory.sOffset();

        // this values represents the position that the new curve should be calculated from (not the current position)
        pos = jmtParams.sOffset();
        Vector2d point = im.prevPoints[used];
        double posFrenet = im.getFrenet(state, point[0], point[1], im.waypoints_[currentLane], *im.spline_[currentLane], im.uniformInterval)[0];

        Vector2d pointAhead;
        double posFrenetAhead;
        if (points.size() > 0) {
            pointAhead = points[points.size() - 1];
            posFrenetAhead = im.getFrenet(state, pointAhead[0], pointAhead[1], im.waypoints_[currentLane], *im.spline_[currentLane], im.uniformInterval)[0];
        }

        printf("Current x, y [%f, %f], state x, y [%f, %f] \n", point[0], point[1], state.x, state.y);
        printf("pos %f, currPos %f, posFrenet %f, posFrenetAhead %f, posState %f, diffFrenet %f, diffCurrFrenet %f, diffFrenetAhead %f, diffState %f\n", pos, currPos, posFrenet, posFrenetAhead, state.s, pos - posFrenet, currPos - posFrenet, pos - posFrenetAhead, pos - state.s);
        printf("old jmt sOffset %f, new jmt sOffset %f\n", im.trajectory.sOffset(), jmtParams.sOffset());

        /*if (fabs(posFrenet - pos) > 0.1) {
            throw std::exception();
        }*/

        pos = posFrenetAhead;
    }

    // determine optimal length
    double remT = im.dT - jmtParams.sCTS().timeToLive();
    double s_start = jmtParams.s().position(0) + pos;
    double s_end = jmtParams.s().position(remT) + pos;
    double s_diff = s_end - s_start;

    //XXX revise d_start
    double d_start = 1.0 * 4.0 + 2.0;

    printf("Pos = %f, s_start %f, s_end %f, s_diff %f\n", pos, s_start, s_end, s_diff);

    // determine optimal path along jmt using original road waypoints
    std::vector<Vector2d> newPoints;
    std::vector<Vector2d> optimalPoints = im.generate_path_points(im.spline_[currentLane], im.waypoints_[currentLane],
                                                                  jmtParams, newPoints, (int) newPoints.size(), 1 * jmtParams.dt(), s_start, s_end, im.s_max_, d_start,
                                                                  targetItems, false);


    double optimalArcLength = arcLength(optimalPoints, 0, optimalPoints.size());

    // add keep points to optimal set
    std::vector<Vector2d> appendedPoints(points);
    appendedPoints.insert(appendedPoints.end(), optimalPoints.begin(), optimalPoints.end());

    double appendedArcLength = arcLength(appendedPoints, 0, appendedPoints.size());

    // generate new scaled path points with appended waypoints from previous curve
    Waypoints scaledWaypoints = generate_scaled_waypoints_from_points(appendedPoints, 0, appendedArcLength, im.uniformInterval, 1.0);

    // generate new localized spline
    std::vector<Vector2d> vScaledWaypoints = xyVectorsToVector2d(scaledWaypoints.x, scaledWaypoints.y);
    SplineType localizedSpline(vScaledWaypoints);

    printf("To Keep points\n");
    for (Vector2d point : points) {
        printf(" [%f, %f], ", point[0], point[1]);
    }

    printf("\nOptimal points\n");
    for (Vector2d point : optimalPoints) {
        printf(" [%f, %f], ", point[0], point[1]);
    }

    printf("\nScaled waypoints\n");
    for (Vector2d point : vScaledWaypoints) {
        printf(" [%f, %f], ", point[0], point[1]);
    }

    double toKeepArcLength = arcLength(points, 0, points.size());
    double scaledWayPointsArcLength = arcLength(vScaledWaypoints, 0, vScaledWaypoints.size());

    // the scaledWaypoints include the 'toKeep' points which have already been scaled, so to generate new scaled points,
    // the offset into the spline must be toKeepArcLength. Also to prevent overlap with the last keep point, the time offset
    // is shifted by one time unit.
    points = im.generate_path_points(&localizedSpline, scaledWaypoints, jmtParams, points, i, 1 * jmtParams.dt(),
                                     toKeepArcLength, scaledWayPointsArcLength, scaledWayPointsArcLength, 0, targetItems, true);
    //points = im.generate_path_points(im.spline_[currentLane], im.waypoints_[currentLane], jmtParams, points, i, 0, s_start, s_end, im.s_max_, d_start, targetItems, true);

    double scaledArcLength = arcLength(points, 0, points.size());

    printf("\nScaled path points\n");
    for (Vector2d point : points) {
        retval.x.push_back(point[0]);
        retval.y.push_back(point[1]);
        printf(" [%f, %f], ", point[0], point[1]);
    }

    printf("s_diff %f, toKeepArcLength %f, optimalArcLength %f, appendedArcLength %f, scaled waypoint arcLength %f, scaledArcLength %f\n", s_diff, toKeepArcLength, optimalArcLength, appendedArcLength, scaledWayPointsArcLength, scaledArcLength);

    if (used > 0) {
        Vector2d& currPoint = im.prevPoints[used - 1];
        Vector2d& hypoPoint = im.prevPoints[used];
        Vector2d& nextPoint = points[0];
        Vector2d& nextPoint2 = points[1];
        double v1 = distance(nextPoint[0], nextPoint[1], currPoint[0], currPoint[1]);
        double v2 = distance(nextPoint[0], nextPoint[1], nextPoint2[0], nextPoint2[1]);
        double hypo2 = distance(currPoint[0], currPoint[1], hypoPoint[0], hypoPoint[1]);
        if (fabs(v2 - v1) > im.dt * 2 && state.speed > 18) {
            throw std::exception();
        }
    }


    printf("complete\n");


#if 0
    printf("Used: %ld\n", used);
    printf("S: %f, Estimated S: %f, diff: %f\n", state.s, JMTeval(im.JMTparams, usedT), state.s - JMTeval(im.JMTparams, usedT));
    printf("Diff between new/old jmt: %f, normal diff: %f\n", diff, normalDiff);
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
                                                                const FTS& fts, std::vector<Vector2d> points,
                                                                int startI, double tOffset, double s_start, double s_end, double s_max,
                                                                double d_start, int maxItems, bool validate) {

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
    double dRemT = fts.sCTS().timeToLive();
    for (j = 0; i < maxItems; i++, j++) {
        double s_delta_eval = fts.s().position(j * fts.dt() + tOffset) + s_start;
        //if (s_delta_eval > s_end + 0.0001) {
        //    break; //end early
        //}
        double s_curr_delta = s_delta_eval - waypoints.s[prevWaypoint];
        double s = (s_curr_delta / s_rate) + prevWaypoint;
        SplineType::InterpolatedPTC curvature = spline->getCurvature(s);
        Eigen::Vector2d normal = getNormalFromCurvature(curvature, s);

        double x = curvature.position[0];
        double y = curvature.position[1];
        double d = d_start;

        //determine d offset
        if (dRemT > 0) {
            double d_delta = fts.d().position(fts.dCTS().offsetTime() + j * fts.dt());
            d += d_delta;
            printf("d_delta %f, j %d, offset %f, d %f\n", d_delta, j, fts.dCTS().offsetTime() + j * fts.dt(), d);
            dRemT -= fts.dt();
        }

        x += normal[0] * d;
        y += normal[1] * d;

        Vector2d point;

        point[0] = x;
        point[1] = y;

        if (points.size() > 0) {
            Vector2d& prevPoint = points[points.size() - 1];
            double dist = distance(point[0], point[1], prevPoint[0], prevPoint[1]);
            double dists = 0;
            if (sv.size() > 0) {
                dists = s_curr_delta - sv[sv.size() - 1];
                if (validate && dists > 25.f*0.02f) {
                    throw new std::exception();
                }
            }
            if (validate && dist > 25 * 0.02f) {
                throw new std::exception();
            }
        }

        if (points.size() > 1) {
            Vector2d& point1 = points[points.size() - 1];
            Vector2d& point2 = points[points.size() - 2];
            double dist1 = distance(point1[0], point1[1], point[0], point[1]);
            double dist2 = distance(point1[0], point1[1], point2[0], point2[1]);
            if (validate && fabs(dist1 - dist2) > 7 * 0.02f) {
                throw new std::exception();
            }
            if (point.dotProduct(point - point1, point1 - point2) < 0) {
                //point is not continuous
                throw new std::exception();
            }
        }

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
        if (trajectory.s().position(t + dt) > s) {
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

VecCollisionBoundingBox PathGenerator::impl::check_overlap(double pos, double d, const std::vector<SensorVehicleState>& sensor_state, const JMT& jmt, double dT, double dt) {
    VecCollisionBoundingBox retval;
    std::vector<CollisionBoundingBox> boundingBoxesCar = calculateLaneBoundingBox(d, pos + jmt.position(0), jmt.speed(0), dT, 4.0);
    for (SensorVehicleState s : sensor_state) {
        std::vector<CollisionBoundingBox> boundingBoxes = calculateLaneBoundingBoxForSensorVehicleState(s, dT, 4.f);

        bool collide = does_overlap(boundingBoxesCar, boundingBoxes);
        if (collide) { retval.push_back(boundingBoxes[0]);
        }
    }
    return retval;
}

VecCollisionBoundingBox PathGenerator::impl::check_collisions(const std::vector<SensorVehicleState>& vehicles,
                                                              const VecCollisionBoundingBox& boxes,
                                                              double d,
                                                              const JMT& params,
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

}


PathGenerator::impl::~impl() {
    for (SplineType* ptr : spline_)
        delete ptr;
}
