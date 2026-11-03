#include "auto_control/passage_geometry.hpp"
#include <cassert>
#include <cmath>

using namespace auto_control::avoidance::passage;

int main()
{
  const Vehicle car{.07,.43,.135};
  // At the BEV rear crop, a 13 cm object is still beside the rear body.
  const Square blind{{-.20,.20},0,.065};
  assert(collision({0,0},0,car,blind,.04));
  assert(!behind_rear_bumper(blind,car,.04));

  // Rear edge and 4 cm reserve must both clear before release.
  const Square edge{{-.50,.40},0,.065};
  const Square clear{{-.54,.40},0,.065};
  assert(!behind_rear_bumper(edge,car,.04));
  assert(behind_rear_bumper(clear,car,.04));
  assert(!collision({0,0},0,car,clear,.04));

  // A yawing car can clip an obstacle despite a safe center-to-center gap.
  const Square corner{{-.27,.27},0,.065};
  assert(!collision({0,0},0,car,corner,.04));
  assert(collision({0,0},-.45,car,corner,.04));

  const auto motion=front_axle_motion(1.0,0.0,.33,.1);
  assert(motion.valid);
  const auto moved=into_new_frame({-.20,.20},motion);
  assert(std::abs(moved.x+.30)<1e-9 && std::abs(moved.y-.20)<1e-9);
  Square tracked=blind;
  for (int frame=1;frame<=3;++frame) {
    tracked.center=into_new_frame(tracked.center,motion);
    assert(!behind_rear_bumper(tracked,car,.04));
  }
  tracked.center=into_new_frame(tracked.center,motion);
  assert(behind_rear_bumper(tracked,car,.04));
  assert(!front_axle_motion(1.0,0.0,.33,.3).valid);
}
