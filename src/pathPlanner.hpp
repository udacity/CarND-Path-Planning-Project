#include <cstdio>
#include <iostream>

#include "helpers.h"
#include "spline.h"

// start in lane 1
auto targetLaneIndex = middle;
auto targetOffsetLat = getLaneDisplacement(targetLaneIndex);
auto currentOffsetLat = targetOffsetLat;
auto offsetLatStep = 0.3;

// reference velocity to target [miles per hour]
double controlSpeed = 0;
const double maxVelocity = 49.5;
double targetSpeed = maxVelocity;
// 5m/s
double velocityStep = 0.224;

void calcNextPoints(points &nextPoints, const egoVehicle &car,
                    const tk::spline &spline, const poseXY &reference) {
  // define the actual (x,y) points we will use for the planer
  // start with all of the previous path points from last time
  for (int i = 0; i < car.previous_path_x.size(); i++) {
    nextPoints.x.push_back(car.previous_path_x[i]);
    nextPoints.y.push_back(car.previous_path_y[i]);
  }

  // Fill up the rest of our path planner after filling it with previous
  // points, here we will always output 50 points
  const int numberOfOutputPoints = 50;
  double x_add_on = 0;
  double distancePerCycle = getTravelledDistance(controlSpeed);
  double delta = distancePerCycle / 1000;
  for (int i = 1; i <= numberOfOutputPoints - car.previous_path_x.size(); i++) {
    double target_dist = 0.0;
    while (target_dist < distancePerCycle) {
      double x1 = x_add_on;
      double y1 = spline(x1);
      x_add_on += delta;
      double y2 = spline(x_add_on);
      target_dist += distance(x1, y1, x_add_on, y2);
    }
    x_add_on -= delta;

    pointXY newPoint;
    newPoint.x = x_add_on;
    newPoint.y = spline(newPoint.x);

    // rotate back to normal after rotating it earlier
    auto newPointTrans = calcRotation(newPoint, reference.yaw);
    newPointTrans = calcTranslation(newPointTrans, reference.xy);

    nextPoints.x.push_back(newPointTrans.x);
    nextPoints.y.push_back(newPointTrans.y);
  }
}

tk::spline calcSpline(poseXY &reference, const egoVehicle &car,
                      const mapWaypoints &map) {
  // reference x,y,yaw state
  // either we will reference the starting point as where the car is or at the
  // previous paths end point
  reference.xy = car.xy;
  reference.yaw = deg2rad(car.yaw);

  // if previous size is almost empty, use the car as starting reference
  pointXY previousPoint;
  if (car.previous_path_x.size() < 2) {
    // use two points that make the path tangent to the car
    previousPoint = calcPreviousPoint(reference.xy, car.yaw);
  }
  // use the previous path's end point end point as starting reference
  else {
    // Redefine reference state as previous path end point
    reference.xy.x = car.previous_path_x[car.previous_path_x.size() - 1];
    reference.xy.y = car.previous_path_y[car.previous_path_x.size() - 1];

    previousPoint.x = car.previous_path_x[car.previous_path_x.size() - 2];
    previousPoint.y = car.previous_path_y[car.previous_path_x.size() - 2];
    reference.yaw = calcYaw(reference.xy, previousPoint);
  }

  // Use two points that make the path tangent to the previous path's end
  // point
  path anchorPoints;
  anchorPoints.xy.push_back(previousPoint);
  anchorPoints.xy.push_back(reference.xy);

  // In frenet add evenly 30m spaced  points ahead of the starting reference
  double startS = 30;
  double stepS = 30;
  double endS = 90;
  for (int offsetLong = startS; offsetLong <= endS; offsetLong += stepS) {
    auto next_wp =
        getXY(car.sd.s + offsetLong, currentOffsetLat, map.s, map.x, map.y);
    anchorPoints.xy.push_back(next_wp);
  }

  vector<double> x;
  vector<double> y;
  for (int i = 0; i < anchorPoints.xy.size(); i++) {
    // shift car reference angle to 0 degrees
    auto shift = calcTranslation(anchorPoints.xy[i], reference.xy, true);
    anchorPoints.xy[i] = calcRotation(shift, 0 - reference.yaw);
    x.push_back(anchorPoints.xy[i].x);
    y.push_back(anchorPoints.xy[i].y);
  }

  // create a spline
  tk::spline spline(x, y);

  return spline;
}

void control(const egoVehicle &car) {
  // rudimentary controlling of velocity
  if (controlSpeed < targetSpeed) {
    controlSpeed += velocityStep;
  } else {
    controlSpeed -= velocityStep;
  }
  if (car.speed > targetSpeed) {
    controlSpeed -= velocityStep;
  }

  // rudimentary control of target lateral offset
  targetOffsetLat = getLaneDisplacement(targetLaneIndex);
  if (currentOffsetLat < targetOffsetLat) {
    currentOffsetLat += offsetLatStep;
  } else {
    currentOffsetLat -= offsetLatStep;
  }
}

void calcLane(egoVehicle &car, vector<vector<double>> sensor_fusion) {
  // place the car at the end of the planned trajectory
  double egoPosition = car.sd.s;
  if (car.previous_path_x.size() > 0) {
    egoPosition = car.end_path_sd.s;
  }

  auto myLane = getLaneDisplacement(targetLaneIndex);

  // find ref_v to use and check if it's within range
  bool too_close = false;
  targetSpeed = maxVelocity;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // car is in my lane
    object obj(sensor_fusion[i]);
    if (obj.sd.d < (myLane + laneWidth / 2) &&
        obj.sd.d > (myLane - laneWidth / 2)) {
      obj.v = distance(0, 0, obj.vx, obj.vx);

      // if using previous points can project s value out
      // check s values greter than mine and s gap
      double objPosition =
          obj.sd.s + ((double)car.previous_path_x.size() * cycleTime * obj.v);

      // predicted target vehicle shall be within certain range
      // 50miles/h -> 22.352m/s
      // using 1 second distance
      double criticalDistance = 22.352;
      if ((objPosition > egoPosition) &&
          ((objPosition - egoPosition) < criticalDistance)) {
        // Do some logic here, lower reference velocity so we dont crash into
        // the car infront of us, could also flag to try to change lanes,
        too_close = true;
        targetSpeed = obj.v;
      }
    }
  }

  // do lane change
  car.currentlaneIndex = static_cast<laneIndex>(car.sd.d / laneWidth);
  if (too_close) {
    // set lane change and wait.
    if (car.currentlaneIndex == targetLaneIndex) {
      // TODO: safe lane change is not ensured
      switch (car.currentlaneIndex) {
        case left:
          targetLaneIndex = middle;
          break;
        case right:
          targetLaneIndex = middle;
          break;
        case middle:
          targetLaneIndex = left;
          break;
        default:
          break;
      }
    }
  }
}

void calc(points &nextPoints, egoVehicle &car, const mapWaypoints &map,
          vector<vector<double>> sensor_fusion) {
  // calc best lane
  calcLane(car, sensor_fusion);

  // rudimentary controlling
  control(car);

  // calc trajectory
  poseXY reference;
  auto spline = calcSpline(reference, car, map);

  // apply trajectory
  calcNextPoints(nextPoints, car, spline, reference);

  // Output debug variables
  std::cout << controlSpeed << ";" << car.speed << ";" << targetSpeed << ";"
            << car.sd.s << ";" << car.sd.d << ";" << car.xy.x << ";" << car.xy.y
            << car.currentlaneIndex << ";" << targetLaneIndex << ";" << car.yaw;

  std::cout << std::endl;
}