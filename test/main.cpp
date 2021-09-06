#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <vector>
#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;

// Declare ur_rtde interfaces
std::shared_ptr<DashboardClient> db_client;
std::shared_ptr<RTDEControlInterface> rtde_control;
std::shared_ptr<RTDEReceiveInterface> rtde_receive;

// Declare initial values
std::vector<double> init_q;
std::vector<double> init_pose;

int main(int argc, char** argv) {
  doctest::Context context;

  context.setOption("abort-after", 5); // stop test execution after 5 failed assertions

  context.applyCommandLine(argc, argv);

  context.setOption("no-breaks", true); // don't break in the debugger when assertions fail

  // Power and brake release the robot through dashboard client
  db_client = std::make_shared<DashboardClient>("192.168.56.101", 29999, true);
  db_client->connect(5000);
  db_client->brakeRelease();

  // Wait for the brakes to release
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Initialize RTDE
  rtde_control = std::make_shared<RTDEControlInterface>("192.168.56.101");
  rtde_receive = std::make_shared<RTDEReceiveInterface>("192.168.56.101");
  init_q = rtde_receive->getActualQ();
  init_pose = rtde_receive->getActualTCPPose();

  int res = context.run(); // run test cases unless with --no-run

  if(context.shouldExit()) // query flags (and --exit) rely on the user doing this
    return res;            // propagate the result of the tests

  return res; // the result from doctest is propagated here as well

  // Stop the RTDE control script
  rtde_control->stopScript();
}

SCENARIO("Move robot in joint space (moveJ)")
{
  GIVEN("A target joint configuration")
  {
    // Move to initial pose, securing that former test dont leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Target is Pi / 6 in the robot base joint
    std::vector<double> target_q = init_q;
    target_q[0] += 0.5235; // ~ Pi / 6

    WHEN("Robot is done moving")
    {
      REQUIRE(rtde_control->moveJ(target_q, 1.05, 1.4));

      THEN("Robot must be at target")
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

SCENARIO("Move robot in tool space (moveL)")
{
  GIVEN("A cartesian target pose")
  {
    // Move to initial pose, securing that former test dont leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Target 10 cm up in the Z-Axis of the TCP
    std::vector<double> target_pose = init_pose;
    target_pose[2] += 0.10; // ~ Pi / 6

    WHEN("Robot is done moving")
    {
      REQUIRE(rtde_control->moveL(target_pose, 0.25, 0.5));

      THEN("Robot must be at target")
      {
        std::vector<double> actual_tcp_pose = rtde_receive->getActualTCPPose();

        for(unsigned int i = 0; i < actual_tcp_pose.size(); i++)
        {
          REQUIRE(actual_tcp_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.005));
        }
      }
    }
  }
}

SCENARIO("Move robot in tool space using a predefined path")
{
  GIVEN("A cartesian target pose")
  {
    // Move to initial pose, securing that former test dont leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Target is defined in this vector
    std::vector<double> target_pose{0.280, -0.400, 0.100, 0, 3.14, 0};

    ur_rtde::Path path;
    double velocity = 0.5;
    double acceleration = 4;

    path.addEntry({PathEntry::MoveJ,
                    PathEntry::PositionTcpPose,
                    {-0.140, -0.400, 0.100, 0, 3.14, 0, velocity, acceleration, 0}});  // move to initial position using movej with inverse kinematics
    path.addEntry({PathEntry::MoveL,
                    PathEntry::PositionTcpPose,
                    {-0.140, -0.400, 0.300, 0, 3.14, 0, velocity, acceleration, 0.099}});
    path.addEntry({PathEntry::MoveL,
                    PathEntry::PositionTcpPose,
                    {-0.140, -0.600, 0.300, 0, 3.14, 0, velocity, acceleration, 0.099}});
    path.addEntry({PathEntry::MoveL,
                    PathEntry::PositionTcpPose,
                    {0.140, -0.600, 0.100, 0, 3.14, 0, velocity, acceleration, 0.099}});
    path.addEntry({PathEntry::MoveL,
                    PathEntry::PositionTcpPose,
                    {0.280, -0.400, 0.100, 0, 3.14, 0, velocity, acceleration, 0}});

    WHEN("Robot is done moving")
    {
      REQUIRE(rtde_control->movePath(path, false));

      THEN("Robot must be at target")
      {
        std::vector<double> actual_tcp_pose = rtde_receive->getActualTCPPose();
        std::cout << "Size of TCPPose from robot is " << actual_tcp_pose.size() << std::endl;
        std::cout << "Actual TCPPose from robot is ";
        for (auto i: actual_tcp_pose)
          std::cout << i << ' ';
        std::cout << std::endl;

        std::vector<double> joint_positions = rtde_receive->getActualQ();
        std::cout << "Size of joint position from robot is " << joint_positions.size() << std::endl;
        std::cout << "Actual joint positions from robot is ";
        for (auto i: joint_positions)
          std::cout << i << ' ';
        std::cout << std::endl;

        for(unsigned int i = 0; i < actual_tcp_pose.size(); i++)
        {
          REQUIRE(actual_tcp_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.005));
        }
      }
    }
  }
}

SCENARIO("Move robot in Forcemode (forceMode)")
{
  GIVEN("Move robot in cartesian space using fixed force")
  {
    // Parameters
    std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
    std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
    std::vector<double> wrench_down = {0, 0, -10, 0, 0, 0};
    std::vector<double> wrench_up = {0, 0, 10, 0, 0, 0};
    int force_type = 2;
    double dt = 1.0/500; // 2ms
    std::vector<double> limits = {2, 2, 1.5, 1, 1, 1};
    std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

    // Move to initial pose, securing that former test dont leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    std::vector<double> start_pose = rtde_receive->getActualTCPPose();

    WHEN("Robot is still moving")
    {
      // Move to initial joint position with a regular moveJ
      REQUIRE(rtde_control->moveJ(joint_q));

      // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
      for (unsigned int i=0; i<2000; i++)
      {
        auto t_start = high_resolution_clock::now();
        // First we move the robot down for 2 seconds, then up for 2 seconds
        if (i > 1000)
          rtde_control->forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
        else
          rtde_control->forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
        auto t_stop = high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);

        if (t_duration.count() < dt)
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
        }
      }
      rtde_control->forceModeStop();

      THEN("Robot must be at differet place that at start")
      {
        std::vector<double> actual_tcp_pose = rtde_receive->getActualTCPPose();

        for(unsigned int i = 0; i < actual_tcp_pose.size(); i++)
        {
          CHECK(actual_tcp_pose[i] != start_pose[i]);
        }
      }
    }
  }
}

SCENARIO("Move robot using servo command (servoJ)")
{
  GIVEN("Servo to position in joint-space) ")
  {
    // Move to initial pose, securing that former test dont leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Parameters
    double velocity = 0.5;
    double acceleration = 0.5;
    double dt = 1.0/500; // 2ms
    double lookahead_time = 0.1;
    double gain = 300;
    std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

    // Target is defined in this vector by trial and error
    std::vector<double> target_pose{0.0897976, -0.390249, 0.1918, -0.768298, 2.76098, -0.74687};

    WHEN("Robot is still moving")
    {
      // Move to initial joint position with a regular moveJ
      REQUIRE(rtde_control->moveJ(joint_q));

      // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
      for (unsigned int i=0; i<1000; i++)
      {
        auto t_start = high_resolution_clock::now();
        rtde_control->servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
        joint_q[0] += 0.001;
        joint_q[1] += 0.001;
        auto t_stop = high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);

        if (t_duration.count() < dt)
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
        }
      }
      rtde_control->servoStop();

      THEN("Robot must be at target")
      {
        std::vector<double> actual_joint_pose = rtde_receive->getActualTCPPose();

        std::cout << "Size of joint position from robot is " << actual_joint_pose.size() << std::endl;
        std::cout << "Actual joint positions from robot is ";
        for (auto i: actual_joint_pose)
          std::cout << i << ' ';
        std::cout << std::endl;

        for(unsigned int i = 0; i < actual_joint_pose.size(); i++)
        {
          CHECK(actual_joint_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.05));

        }
      }
    }
  }
}

SCENARIO("Move robot using SpeedJ command (SpeedJ)")
{
  GIVEN("Servo to position in joint-space) ")
  {
    // Move to initial pose, securing that former test dont leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Parameters
    double acceleration = 0.5;
    double dt = 1.0/500; // 2ms
    std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Target is defined in this vector by trial and error
    std::vector<double> target_pose{0.0897976, -0.390249, 0.1918, -0.768298, 2.76098, -0.74687};

    WHEN("Robot is still moving")
    {
      // Move to initial joint position with a regular moveJ
      REQUIRE(rtde_control->moveJ(joint_q));

      // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
      for (unsigned int i=0; i<1000; i++)
      {
        auto t_start = high_resolution_clock::now();
        rtde_control->speedJ(joint_speed, acceleration, dt);
        joint_speed[0] += 0.0005;
        joint_speed[1] += 0.0005;
        auto t_stop = high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);

        if (t_duration.count() < dt)
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
        }
      }
      rtde_control->speedStop();

      THEN("Robot must be at target")
      {
        std::vector<double> actual_joint_pose = rtde_receive->getActualTCPPose();

        std::cout << "Size of joint position from robot is " << actual_joint_pose.size() << std::endl;
        std::cout << "Actual joint positions from robot is ";
        for (auto i: actual_joint_pose)
          std::cout << i << ' ';
        std::cout << std::endl;

        for(unsigned int i = 0; i < actual_joint_pose.size(); i++)
        {
          CHECK(actual_joint_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.05));

        }
      }
    }
  }
}
