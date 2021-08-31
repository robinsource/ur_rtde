#include "test-case-01.h"

SCENARIO("move robot in joint space (moveJ)")
{
  GIVEN("a target joint configuration")
  {
    // Target is Pi / 6 in the robot base joint
    std::vector<double> target_q = init_q;
    target_q[0] += 0.5235; // ~ Pi / 6

    WHEN("robot is done moving")
    {
      REQUIRE(rtde_control->moveJ(target_q, 1.05, 1.4));

      THEN("robot must be at target")
      {
        std::vector<double> actual_q = rtde_receive->getActualQ();

        for(unsigned int i = 0; i < actual_q.size(); i++)
        {
          REQUIRE(actual_q[i] == doctest::Approx(target_q[i]).epsilon(0.005));
        }
      }
    }
  }
}