#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace auto_control::avoidance::passage
{
struct Point
{
  double x{0.0}, y{0.0};
};

struct Motion
{
  Point translation;  // New front-axle origin in the previous vehicle frame.
  double yaw{0.0};     // New vehicle heading relative to the previous frame.
  double distance{0.0};
  bool valid{false};
};

struct Vehicle
{
  double front{0.07};       // Front axle to front bumper.
  double rear{0.43};        // Front axle to rear bumper.
  double half_width{0.135};
};

struct Square
{
  Point center;
  double yaw{0.0};
  double half_size{0.065};
};

inline Point rotate(Point p, double angle)
{
  const double c = std::cos(angle), s = std::sin(angle);
  return {c*p.x-s*p.y, s*p.x+c*p.y};
}

inline Point into_new_frame(Point point, const Motion & motion)
{
  return rotate({point.x-motion.translation.x, point.y-motion.translation.y}, -motion.yaw);
}

// Rear-axle speed and measured yaw rate determine front-axle movement.
inline Motion front_axle_motion(double speed, double yaw_rate, double wheelbase, double dt)
{
  if (!std::isfinite(speed) || !std::isfinite(yaw_rate) || !std::isfinite(wheelbase) ||
      !std::isfinite(dt) || speed < 0.0 || wheelbase <= 0.0 || dt < 0.0 || dt > 0.25)
  {return {};}
  const double yaw = yaw_rate*dt;
  const double distance = speed*dt;
  if (std::abs(yaw_rate) < 1e-8) {return {{distance,0.0},yaw,distance,true};}
  const double radius = speed/yaw_rate;
  return {{radius*std::sin(yaw)+wheelbase*(std::cos(yaw)-1.0),
           radius*(1.0-std::cos(yaw))+wheelbase*std::sin(yaw)},yaw,distance,true};
}

inline std::array<Point,4> vehicle_corners(Point axle, double heading, const Vehicle & vehicle,
                                           double margin = 0.0)
{
  const std::array<Point,4> local{{
    {vehicle.front+margin,vehicle.half_width+margin},
    {vehicle.front+margin,-vehicle.half_width-margin},
    {-vehicle.rear-margin,-vehicle.half_width-margin},
    {-vehicle.rear-margin,vehicle.half_width+margin}}};
  std::array<Point,4> result;
  for (std::size_t i=0;i<4;++i) {
    const auto r=rotate(local[i],heading);
    result[i]={axle.x+r.x,axle.y+r.y};
  }
  return result;
}

inline std::array<Point,4> square_corners(const Square & square)
{
  const double h=square.half_size;
  const std::array<Point,4> local{{{h,h},{h,-h},{-h,-h},{-h,h}}};
  std::array<Point,4> result;
  for (std::size_t i=0;i<4;++i) {
    const auto r=rotate(local[i],square.yaw);
    result[i]={square.center.x+r.x,square.center.y+r.y};
  }
  return result;
}

inline bool overlap(const std::array<Point,4> & a, const std::array<Point,4> & b)
{
  const std::array<Point,4> axes{{
    {a[1].y-a[0].y,a[0].x-a[1].x},
    {a[2].y-a[1].y,a[1].x-a[2].x},
    {b[1].y-b[0].y,b[0].x-b[1].x},
    {b[2].y-b[1].y,b[1].x-b[2].x}}};
  for (const auto axis:axes) {
    double amin=1e100,amax=-1e100,bmin=1e100,bmax=-1e100;
    for (const auto p:a) {const double v=p.x*axis.x+p.y*axis.y; amin=std::min(amin,v); amax=std::max(amax,v);}
    for (const auto p:b) {const double v=p.x*axis.x+p.y*axis.y; bmin=std::min(bmin,v); bmax=std::max(bmax,v);}
    if (amax < bmin || bmax < amin) {return false;}
  }
  return true;
}

inline bool collision(Point axle, double heading, const Vehicle & vehicle,
                      const Square & obstacle, double margin)
{
  return overlap(vehicle_corners(axle,heading,vehicle,margin),square_corners(obstacle));
}

inline bool behind_rear_bumper(const Square & obstacle, const Vehicle & vehicle, double margin)
{
  const double extent=obstacle.half_size*(std::abs(std::cos(obstacle.yaw))+
                                           std::abs(std::sin(obstacle.yaw)));
  return obstacle.center.x+extent < -vehicle.rear-margin;
}
}  // namespace auto_control::avoidance::passage
