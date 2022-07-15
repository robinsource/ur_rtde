#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;

// interrupt flag
bool running = true;
void raiseFlag(int param)
{
  running = false;
}

std::vector<double> getCircleTarget(const std::vector<double> &pose, double timestep, double radius=5, double freq=5,
                                    double offset_x=0.075, double offset_y=0.075)
{
  std::vector<double> circ_target = pose;
  circ_target[0] = pose[0] + offset_x * cos((2 * M_PI * freq * timestep) / radius);
  circ_target[1] = pose[1] + offset_y * sin((2 * M_PI * freq * timestep) / radius);
  return circ_target;
}

int main(int argc, char* argv[])
{
  // Setup parameters
  std::string robot_ip = "192.168.1.129";
  double rtde_frequency = 500.0; // Hz
  double dt = 1.0 / rtde_frequency; // 2ms
  uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
  int ur_cap_port = 50002;

  // ur_rtde realtime priorities
  int rt_receive_priority = 90; // 90
  int rt_control_priority = 85; // 85

  RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
  RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);

  // Set application realtime priority
  RTDEUtility::setRealtimePriority(80); // 80

  // Move parameters
  double vel = 0.5;
  double acc = 0.5;

  // Servo control parameters
  double lookahead_time = 0.1;
  double gain = 600;

  signal(SIGINT, raiseFlag);

  double time_counter = 0.0;
  bool initial_run = true;
  int missed_deadline_count = 0;
  std::vector<double> cycle_times;

  // Move to init position using moveL
  std::vector<double> actual_tcp_pose = rtde_receive.getActualTCPPose();
  std::vector<double> init_pose = getCircleTarget(actual_tcp_pose, time_counter);
  rtde_control.moveL(init_pose, vel, acc);

  try
  {
    while (running)
    {
      auto t_cycle_start = steady_clock::now();

      rtde_control.initPeriod();
      std::vector<double> servo_target = getCircleTarget(actual_tcp_pose, time_counter);
      rtde_control.servoL(servo_target, vel, acc, dt, lookahead_time, gain);
      rtde_control.waitPeriod(dt);

      auto t_cycle_stop = steady_clock::now();
      auto t_cycle_duration = std::chrono::duration<double>(t_cycle_stop - t_cycle_start);
      if (!initial_run)
        cycle_times.push_back(t_cycle_duration.count());

      initial_run = false;
      time_counter += dt;
    }

    std::cout << "Control interrupted!" << std::endl;
    rtde_control.servoStop();
    rtde_control.stopScript();
    std::ofstream data_recording("rt_cycle_times.csv");
    data_recording << std::fixed << std::setprecision(6);
    data_recording << "time" << "," << "control_cycle_time" << '\n';
    int sample = 0;
    for (const auto &var : cycle_times)
    {
      data_recording << sample << "," << var << '\n';
      sample++;
    }
    data_recording.close();
  }
  catch(std::exception& e)
  {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  catch(...)
  {
    std::cerr << "Exception of unknown type!\n";
  }
  return 0;
}
