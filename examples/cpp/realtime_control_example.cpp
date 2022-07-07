#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_utility.h>
#include <thread>
#include <chrono>
#include <sys/mman.h>

using namespace ur_rtde;
using namespace std::chrono;

// Interrupt flag
bool flag_loop = true;
void raiseFlag(int param)
{
  flag_loop = false;
}

double lerp(double a, double b, double t)
{
  return a + t * (b - a);
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
  int rt_receive_priority = 90;
  int rt_control_priority = 85;

  RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);
  RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);

  // Set application realtime priority
  RTDEUtility::setRealtimePriority(80);

  // Move parameters
  double vel = 0.5;
  double acc = 0.5;

  // Servo control parameters
  double lookahead_time = 0.1;
  double gain = 600;

  // Define targets
  std::vector<double> actual_q = rtde_receive.getActualQ();
  double base_target_0 = actual_q[0]-0.4;
  double base_target_1 = actual_q[0]+0.4;

  signal(SIGINT, raiseFlag);

  double time_counter = 0.0;
  bool initial_run = true;
  bool counting_up = true;
  int missed_deadline_count = 0;

  // Move to init position using moveJ
  std::vector<double> init_position = actual_q;
  init_position[0] = base_target_0;
  rtde_control.moveJ(init_position, vel, acc);

  try
  {
    // Execute 500Hz realtime control loop
    while (flag_loop)
    {
      auto t_start = steady_clock::now();
      double base_target = lerp(base_target_0, base_target_1, time_counter);
      std::vector<double> servo_target = actual_q;
      servo_target[0] = base_target;
      rtde_control.servoJ(servo_target, vel, acc, dt, lookahead_time, gain);

      auto t_stop = steady_clock::now();
      auto t_duration = std::chrono::duration<double>(t_stop - t_start);
      if (t_duration.count() < dt)
      {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
      }
      if (t_duration.count() > dt)
      {
        if (!initial_run)
        {
          std::cout << "Realtime application control loop exceeds 2ms!" << std::endl;
          missed_deadline_count++;
          std::cout << "ur_rtde: missed deadlines: " << missed_deadline_count << std::endl;
        }
      }

      if (counting_up)
      {
        time_counter += dt;
        if (time_counter >= 0.5)
          counting_up = false;
      }
      else
      {
        time_counter -= dt;
        if (time_counter <= 0.0)
          counting_up = true;
      }
      initial_run = false;
    }

    std::cout << "Control interrupted!" << std::endl;
    rtde_control.servoStop();
    rtde_control.stopScript();
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
