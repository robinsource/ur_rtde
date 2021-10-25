#pragma once
#ifndef RTDE_ROBOT_STATE_H
#define RTDE_ROBOT_STATE_H

#include <ur_rtde/rtde_export.h>
#include <vector>
#include <cstdint>
#include <mutex>

namespace ur_rtde
{
class RobotState
{
 public:
  RTDE_EXPORT explicit RobotState();

  RTDE_EXPORT virtual ~RobotState();

  RTDE_EXPORT bool lockUpdateStateMutex();

  RTDE_EXPORT bool unlockUpdateStateMutex();

 public:
  RTDE_EXPORT double getTimestamp() const;
  RTDE_EXPORT void setTimestamp(double timestamp);
  RTDE_EXPORT const std::vector<double> getTarget_q() const;
  RTDE_EXPORT void setTarget_q(const std::vector<double> &target_q);
  RTDE_EXPORT const std::vector<double> getTarget_qd() const;
  RTDE_EXPORT void setTarget_qd(const std::vector<double> &target_qd);
  RTDE_EXPORT const std::vector<double> getTarget_qdd() const;
  RTDE_EXPORT void setTarget_qdd(const std::vector<double> &target_qdd);
  RTDE_EXPORT const std::vector<double> getTarget_current() const;
  RTDE_EXPORT void setTarget_current(const std::vector<double> &target_current);
  RTDE_EXPORT const std::vector<double> getTarget_moment() const;
  RTDE_EXPORT void setTarget_moment(const std::vector<double> &target_moment);
  RTDE_EXPORT const std::vector<double> getActual_q() const;
  RTDE_EXPORT void setActual_q(const std::vector<double> &actual_q);
  RTDE_EXPORT const std::vector<double> getActual_qd() const;
  RTDE_EXPORT void setActual_qd(const std::vector<double> &actual_qd);
  RTDE_EXPORT const std::vector<double> getActual_current() const;
  RTDE_EXPORT void setActual_current(const std::vector<double> &actual_current);
  RTDE_EXPORT const std::vector<double> getJoint_control_output() const;
  RTDE_EXPORT void setJoint_control_output(const std::vector<double> &joint_control_output);
  RTDE_EXPORT const std::vector<double> getActual_TCP_pose() const;
  RTDE_EXPORT void setActual_TCP_pose(const std::vector<double> &actual_TCP_pose);
  RTDE_EXPORT const std::vector<double> getActual_TCP_speed() const;
  RTDE_EXPORT void setActual_TCP_speed(const std::vector<double> &actual_TCP_speed);
  RTDE_EXPORT const std::vector<double> getActual_TCP_force() const;
  RTDE_EXPORT void setActual_TCP_force(const std::vector<double> &actual_TCP_force);
  RTDE_EXPORT const std::vector<double> getTarget_TCP_pose() const;
  RTDE_EXPORT void setTarget_TCP_pose(const std::vector<double> &target_TCP_pose);
  RTDE_EXPORT const std::vector<double> getTarget_TCP_speed() const;
  RTDE_EXPORT void setTarget_TCP_speed(const std::vector<double> &target_TCP_speed);
  RTDE_EXPORT uint64_t getActual_digital_input_bits() const;
  RTDE_EXPORT void setActual_digital_input_bits(uint64_t actual_digital_input_bits);
  RTDE_EXPORT const std::vector<double> getJoint_temperatures() const;
  RTDE_EXPORT void setJoint_temperatures(const std::vector<double> &joint_temperatures);
  RTDE_EXPORT double getActual_execution_time() const;
  RTDE_EXPORT void setActual_execution_time(double actual_execution_time);
  RTDE_EXPORT int32_t getRobot_mode() const;
  RTDE_EXPORT void setRobot_mode(int32_t robot_mode);
  RTDE_EXPORT uint32_t getRobot_status() const;
  RTDE_EXPORT void setRobot_status(uint32_t robot_status);
  RTDE_EXPORT uint32_t getSafety_status_bits() const;
  RTDE_EXPORT void setSafety_status_bits(uint32_t safety_status_bits);
  RTDE_EXPORT const std::vector<int32_t> getJoint_mode() const;
  RTDE_EXPORT void setJoint_mode(const std::vector<int32_t> &joint_mode);
  RTDE_EXPORT int32_t getSafety_mode() const;
  RTDE_EXPORT void setSafety_mode(int32_t safety_mode);
  RTDE_EXPORT const std::vector<double> getActual_tool_accelerometer() const;
  RTDE_EXPORT void setActual_tool_accelerometer(const std::vector<double> &actual_tool_accelerometer);
  RTDE_EXPORT double getSpeed_scaling() const;
  RTDE_EXPORT void setSpeed_scaling(double speed_scaling);
  RTDE_EXPORT double getTarget_speed_fraction() const;
  RTDE_EXPORT void setTarget_speed_fraction(double target_speed_fraction);
  RTDE_EXPORT double getActual_momentum() const;
  RTDE_EXPORT void setActual_momentum(double actual_momentum);
  RTDE_EXPORT double getActual_main_voltage() const;
  RTDE_EXPORT void setActual_main_voltage(double actual_main_voltage);
  RTDE_EXPORT double getActual_robot_voltage() const;
  RTDE_EXPORT void setActual_robot_voltage(double actual_robot_voltage);
  RTDE_EXPORT double getActual_robot_current() const;
  RTDE_EXPORT void setActual_robot_current(double actual_robot_current);
  RTDE_EXPORT const std::vector<double> getActual_joint_voltage() const;
  RTDE_EXPORT void setActual_joint_voltage(const std::vector<double> &actual_joint_voltage);
  RTDE_EXPORT uint64_t getActual_digital_output_bits() const;
  RTDE_EXPORT void setActual_digital_output_bits(uint64_t actual_digital_output_bits);
  RTDE_EXPORT uint32_t getRuntime_state() const;
  RTDE_EXPORT double getStandard_analog_input_0() const;
  RTDE_EXPORT void setStandard_analog_input_0(double standard_analog_input_0);
  RTDE_EXPORT double getStandard_analog_input_1() const;
  RTDE_EXPORT void setStandard_analog_input_1(double standard_analog_input_1);
  RTDE_EXPORT double getStandard_analog_output_0() const;
  RTDE_EXPORT void setStandard_analog_output_0(double standard_analog_output_0);
  RTDE_EXPORT double getStandard_analog_output_1() const;
  RTDE_EXPORT void setStandard_analog_output_1(double standard_analog_output_1);
  RTDE_EXPORT void setRuntime_state(uint32_t runtime_state);
  RTDE_EXPORT uint32_t getOutput_bit_registers0_to_31() const;
  RTDE_EXPORT void setOutput_bit_registers0_to_31(uint32_t output_bit_registers0_to_31);
  RTDE_EXPORT uint32_t getOutput_bit_registers32_to_63() const;
  RTDE_EXPORT void setOutput_bit_registers32_to_63(uint32_t output_bit_registers32_to_63);

  int32_t getOutput_int_register(int n) const {
    std::lock_guard<std::mutex> lock(update_state_mutex_);
    return output_int_registers_[n];
  }
  void setOutput_int_register(int n, int32_t val) {
    output_int_registers_[n] = val;
  }
  double getOutput_double_register(int n) const {
    std::lock_guard<std::mutex> lock(update_state_mutex_);
    return output_double_registers_[n];
  }
  void setOutput_double_register(int n, double val) {
    output_double_registers_[n] = val;
  }

  RTDE_EXPORT int32_t getOutput_int_register_0() const;
  RTDE_EXPORT void setOutput_int_register_0(int32_t output_int_register_0);
  RTDE_EXPORT int32_t getOutput_int_register_1() const;
  RTDE_EXPORT void setOutput_int_register_1(int32_t output_int_register_1);
  RTDE_EXPORT int32_t getOutput_int_register_2() const;
  RTDE_EXPORT void setOutput_int_register_2(int32_t output_int_register_2);
  RTDE_EXPORT int32_t getOutput_int_register_3() const;
  RTDE_EXPORT void setOutput_int_register_3(int32_t output_int_register_3);
  RTDE_EXPORT int32_t getOutput_int_register_4() const;
  RTDE_EXPORT void setOutput_int_register_4(int32_t output_int_register_4);
  RTDE_EXPORT int32_t getOutput_int_register_5() const;
  RTDE_EXPORT void setOutput_int_register_5(int32_t output_int_register_5);
  RTDE_EXPORT int32_t getOutput_int_register_6() const;
  RTDE_EXPORT void setOutput_int_register_6(int32_t output_int_register_6);
  RTDE_EXPORT int32_t getOutput_int_register_7() const;
  RTDE_EXPORT void setOutput_int_register_7(int32_t output_int_register_7);
  RTDE_EXPORT int32_t getOutput_int_register_8() const;
  RTDE_EXPORT void setOutput_int_register_8(int32_t output_int_register_8);
  RTDE_EXPORT int32_t getOutput_int_register_9() const;
  RTDE_EXPORT void setOutput_int_register_9(int32_t output_int_register_9);
  RTDE_EXPORT int32_t getOutput_int_register_10() const;
  RTDE_EXPORT void setOutput_int_register_10(int32_t output_int_register_10);
  RTDE_EXPORT int32_t getOutput_int_register_11() const;
  RTDE_EXPORT void setOutput_int_register_11(int32_t output_int_register_11);
  RTDE_EXPORT int32_t getOutput_int_register_12() const;
  RTDE_EXPORT void setOutput_int_register_12(int32_t output_int_register_12);
  RTDE_EXPORT int32_t getOutput_int_register_13() const;
  RTDE_EXPORT void setOutput_int_register_13(int32_t output_int_register_13);
  RTDE_EXPORT int32_t getOutput_int_register_14() const;
  RTDE_EXPORT void setOutput_int_register_14(int32_t output_int_register_14);
  RTDE_EXPORT int32_t getOutput_int_register_15() const;
  RTDE_EXPORT void setOutput_int_register_15(int32_t output_int_register_15);
  RTDE_EXPORT int32_t getOutput_int_register_16() const;
  RTDE_EXPORT void setOutput_int_register_16(int32_t output_int_register_16);
  RTDE_EXPORT int32_t getOutput_int_register_17() const;
  RTDE_EXPORT void setOutput_int_register_17(int32_t output_int_register_17);
  RTDE_EXPORT int32_t getOutput_int_register_18() const;
  RTDE_EXPORT void setOutput_int_register_18(int32_t output_int_register_18);
  RTDE_EXPORT int32_t getOutput_int_register_19() const;
  RTDE_EXPORT void setOutput_int_register_19(int32_t output_int_register_19);
  RTDE_EXPORT int32_t getOutput_int_register_20() const;
  RTDE_EXPORT void setOutput_int_register_20(int32_t output_int_register_20);
  RTDE_EXPORT int32_t getOutput_int_register_21() const;
  RTDE_EXPORT void setOutput_int_register_21(int32_t output_int_register_21);
  RTDE_EXPORT int32_t getOutput_int_register_22() const;
  RTDE_EXPORT void setOutput_int_register_22(int32_t output_int_register_22);
  RTDE_EXPORT int32_t getOutput_int_register_23() const;
  RTDE_EXPORT void setOutput_int_register_23(int32_t output_int_register_23);

  RTDE_EXPORT int32_t getOutput_int_register_24() const;
  RTDE_EXPORT void setOutput_int_register_24(int32_t output_int_register_24);
  RTDE_EXPORT int32_t getOutput_int_register_25() const;
  RTDE_EXPORT void setOutput_int_register_25(int32_t output_int_register_25);
  RTDE_EXPORT int32_t getOutput_int_register_26() const;
  RTDE_EXPORT void setOutput_int_register_26(int32_t output_int_register_26);
  RTDE_EXPORT int32_t getOutput_int_register_27() const;
  RTDE_EXPORT void setOutput_int_register_27(int32_t output_int_register_27);
  RTDE_EXPORT int32_t getOutput_int_register_28() const;
  RTDE_EXPORT void setOutput_int_register_28(int32_t output_int_register_28);
  RTDE_EXPORT int32_t getOutput_int_register_29() const;
  RTDE_EXPORT void setOutput_int_register_29(int32_t output_int_register_29);
  RTDE_EXPORT int32_t getOutput_int_register_30() const;
  RTDE_EXPORT void setOutput_int_register_30(int32_t output_int_register_30);
  RTDE_EXPORT int32_t getOutput_int_register_31() const;
  RTDE_EXPORT void setOutput_int_register_31(int32_t output_int_register_31);
  RTDE_EXPORT int32_t getOutput_int_register_32() const;
  RTDE_EXPORT void setOutput_int_register_32(int32_t output_int_register_32);
  RTDE_EXPORT int32_t getOutput_int_register_33() const;
  RTDE_EXPORT void setOutput_int_register_33(int32_t output_int_register_33);
  RTDE_EXPORT int32_t getOutput_int_register_34() const;
  RTDE_EXPORT void setOutput_int_register_34(int32_t output_int_register_24);
  RTDE_EXPORT int32_t getOutput_int_register_35() const;
  RTDE_EXPORT void setOutput_int_register_35(int32_t output_int_register_35);
  RTDE_EXPORT int32_t getOutput_int_register_36() const;
  RTDE_EXPORT void setOutput_int_register_36(int32_t output_int_register_36);
  RTDE_EXPORT int32_t getOutput_int_register_37() const;
  RTDE_EXPORT void setOutput_int_register_37(int32_t output_int_register_37);
  RTDE_EXPORT int32_t getOutput_int_register_38() const;
  RTDE_EXPORT void setOutput_int_register_38(int32_t output_int_register_38);
  RTDE_EXPORT int32_t getOutput_int_register_39() const;
  RTDE_EXPORT void setOutput_int_register_39(int32_t output_int_register_39);
  RTDE_EXPORT int32_t getOutput_int_register_40() const;
  RTDE_EXPORT void setOutput_int_register_40(int32_t output_int_register_40);
  RTDE_EXPORT int32_t getOutput_int_register_41() const;
  RTDE_EXPORT void setOutput_int_register_41(int32_t output_int_register_41);
  RTDE_EXPORT int32_t getOutput_int_register_42() const;
  RTDE_EXPORT void setOutput_int_register_42(int32_t output_int_register_42);
  RTDE_EXPORT int32_t getOutput_int_register_43() const;
  RTDE_EXPORT void setOutput_int_register_43(int32_t output_int_register_43);
  RTDE_EXPORT int32_t getOutput_int_register_44() const;
  RTDE_EXPORT void setOutput_int_register_44(int32_t output_int_register_44);
  RTDE_EXPORT int32_t getOutput_int_register_45() const;
  RTDE_EXPORT void setOutput_int_register_45(int32_t output_int_register_45);
  RTDE_EXPORT int32_t getOutput_int_register_46() const;
  RTDE_EXPORT void setOutput_int_register_46(int32_t output_int_register_46);
  RTDE_EXPORT int32_t getOutput_int_register_47() const;
  RTDE_EXPORT void setOutput_int_register_47(int32_t output_int_register_47);

  RTDE_EXPORT double getOutput_double_register_0() const;
  RTDE_EXPORT void setOutput_double_register_0(double output_double_register_0);
  RTDE_EXPORT double getOutput_double_register_1() const;
  RTDE_EXPORT void setOutput_double_register_1(double output_double_register_1);
  RTDE_EXPORT double getOutput_double_register_2() const;
  RTDE_EXPORT void setOutput_double_register_2(double output_double_register_2);
  RTDE_EXPORT double getOutput_double_register_3() const;
  RTDE_EXPORT void setOutput_double_register_3(double output_double_register_3);
  RTDE_EXPORT double getOutput_double_register_4() const;
  RTDE_EXPORT void setOutput_double_register_4(double output_double_register_4);
  RTDE_EXPORT double getOutput_double_register_5() const;
  RTDE_EXPORT void setOutput_double_register_5(double output_double_register_5);
  RTDE_EXPORT double getOutput_double_register_6() const;
  RTDE_EXPORT void setOutput_double_register_6(double output_double_register_6);
  RTDE_EXPORT double getOutput_double_register_7() const;
  RTDE_EXPORT void setOutput_double_register_7(double output_double_register_7);
  RTDE_EXPORT double getOutput_double_register_8() const;
  RTDE_EXPORT void setOutput_double_register_8(double output_double_register_8);
  RTDE_EXPORT double getOutput_double_register_9() const;
  RTDE_EXPORT void setOutput_double_register_9(double output_double_register_9);
  RTDE_EXPORT double getOutput_double_register_10() const;
  RTDE_EXPORT void setOutput_double_register_10(double output_double_register_10);
  RTDE_EXPORT double getOutput_double_register_11() const;
  RTDE_EXPORT void setOutput_double_register_11(double output_double_register_11);
  RTDE_EXPORT double getOutput_double_register_12() const;
  RTDE_EXPORT void setOutput_double_register_12(double output_double_register_12);
  RTDE_EXPORT double getOutput_double_register_13() const;
  RTDE_EXPORT void setOutput_double_register_13(double output_double_register_13);
  RTDE_EXPORT double getOutput_double_register_14() const;
  RTDE_EXPORT void setOutput_double_register_14(double output_double_register_14);
  RTDE_EXPORT double getOutput_double_register_15() const;
  RTDE_EXPORT void setOutput_double_register_15(double output_double_register_15);
  RTDE_EXPORT double getOutput_double_register_16() const;
  RTDE_EXPORT void setOutput_double_register_16(double output_double_register_16);
  RTDE_EXPORT double getOutput_double_register_17() const;
  RTDE_EXPORT void setOutput_double_register_17(double output_double_register_17);
  RTDE_EXPORT double getOutput_double_register_18() const;
  RTDE_EXPORT void setOutput_double_register_18(double output_double_register_18);
  RTDE_EXPORT double getOutput_double_register_19() const;
  RTDE_EXPORT void setOutput_double_register_19(double output_double_register_19);
  RTDE_EXPORT double getOutput_double_register_20() const;
  RTDE_EXPORT void setOutput_double_register_20(double output_double_register_20);
  RTDE_EXPORT double getOutput_double_register_21() const;
  RTDE_EXPORT void setOutput_double_register_21(double output_double_register_21);
  RTDE_EXPORT double getOutput_double_register_22() const;
  RTDE_EXPORT void setOutput_double_register_22(double output_double_register_22);
  RTDE_EXPORT double getOutput_double_register_23() const;
  RTDE_EXPORT void setOutput_double_register_23(double output_double_register_23);

  RTDE_EXPORT double getOutput_double_register_24() const;
  RTDE_EXPORT void setOutput_double_register_24(double output_double_register_24);
  RTDE_EXPORT double getOutput_double_register_25() const;
  RTDE_EXPORT void setOutput_double_register_25(double output_double_register_25);
  RTDE_EXPORT double getOutput_double_register_26() const;
  RTDE_EXPORT void setOutput_double_register_26(double output_double_register_26);
  RTDE_EXPORT double getOutput_double_register_27() const;
  RTDE_EXPORT void setOutput_double_register_27(double output_double_register_27);
  RTDE_EXPORT double getOutput_double_register_28() const;
  RTDE_EXPORT void setOutput_double_register_28(double output_double_register_28);
  RTDE_EXPORT double getOutput_double_register_29() const;
  RTDE_EXPORT void setOutput_double_register_29(double output_double_register_29);
  RTDE_EXPORT double getOutput_double_register_30() const;
  RTDE_EXPORT void setOutput_double_register_30(double output_double_register_30);
  RTDE_EXPORT double getOutput_double_register_31() const;
  RTDE_EXPORT void setOutput_double_register_31(double output_double_register_31);
  RTDE_EXPORT double getOutput_double_register_32() const;
  RTDE_EXPORT void setOutput_double_register_32(double output_double_register_32);
  RTDE_EXPORT double getOutput_double_register_33() const;
  RTDE_EXPORT void setOutput_double_register_33(double output_double_register_33);
  RTDE_EXPORT double getOutput_double_register_34() const;
  RTDE_EXPORT void setOutput_double_register_34(double output_double_register_34);
  RTDE_EXPORT double getOutput_double_register_35() const;
  RTDE_EXPORT void setOutput_double_register_35(double output_double_register_35);
  RTDE_EXPORT double getOutput_double_register_36() const;
  RTDE_EXPORT void setOutput_double_register_36(double output_double_register_36);
  RTDE_EXPORT double getOutput_double_register_37() const;
  RTDE_EXPORT void setOutput_double_register_37(double output_double_register_37);
  RTDE_EXPORT double getOutput_double_register_38() const;
  RTDE_EXPORT void setOutput_double_register_38(double output_double_register_38);
  RTDE_EXPORT double getOutput_double_register_39() const;
  RTDE_EXPORT void setOutput_double_register_39(double output_double_register_39);
  RTDE_EXPORT double getOutput_double_register_40() const;
  RTDE_EXPORT void setOutput_double_register_40(double output_double_register_40);
  RTDE_EXPORT double getOutput_double_register_41() const;
  RTDE_EXPORT void setOutput_double_register_41(double output_double_register_41);
  RTDE_EXPORT double getOutput_double_register_42() const;
  RTDE_EXPORT void setOutput_double_register_42(double output_double_register_42);
  RTDE_EXPORT double getOutput_double_register_43() const;
  RTDE_EXPORT void setOutput_double_register_43(double output_double_register_43);
  RTDE_EXPORT double getOutput_double_register_44() const;
  RTDE_EXPORT void setOutput_double_register_44(double output_double_register_44);
  RTDE_EXPORT double getOutput_double_register_45() const;
  RTDE_EXPORT void setOutput_double_register_45(double output_double_register_45);
  RTDE_EXPORT double getOutput_double_register_46() const;
  RTDE_EXPORT void setOutput_double_register_46(double output_double_register_46);
  RTDE_EXPORT double getOutput_double_register_47() const;
  RTDE_EXPORT void setOutput_double_register_47(double output_double_register_47);

 private:
  double timestamp_;
  std::vector<double> target_q_;
  std::vector<double> target_qd_;
  std::vector<double> target_qdd_;
  std::vector<double> target_current_;
  std::vector<double> target_moment_;
  std::vector<double> actual_q_;
  std::vector<double> actual_qd_;
  std::vector<double> actual_current_;
  std::vector<double> joint_control_output_;
  std::vector<double> actual_TCP_pose_;
  std::vector<double> actual_TCP_speed_;
  std::vector<double> actual_TCP_force_;
  std::vector<double> target_TCP_pose_;
  std::vector<double> target_TCP_speed_;
  uint64_t actual_digital_input_bits_;
  std::vector<double> joint_temperatures_;
  double actual_execution_time_;
  int32_t robot_mode_;
  uint32_t robot_status_;
  uint32_t safety_status_bits_;
  std::vector<int32_t> joint_mode_;
  int32_t safety_mode_;
  std::vector<double> actual_tool_accelerometer_;
  double speed_scaling_;
  double target_speed_fraction_;
  double actual_momentum_;
  double actual_main_voltage_;
  double actual_robot_voltage_;
  double actual_robot_current_;
  std::vector<double> actual_joint_voltage_;
  uint64_t actual_digital_output_bits_;
  uint32_t runtime_state_;
  double standard_analog_input_0_;
  double standard_analog_input_1_;
  double standard_analog_output_0_;
  double standard_analog_output_1_;

  uint32_t output_bit_registers0_to_31_;
  uint32_t output_bit_registers32_to_63_;
  int32_t output_int_registers_[48];
  double output_double_registers_[48];

  mutable std::mutex update_state_mutex_;
};

}  // namespace ur_rtde

#endif  // RTDE_ROBOT_STATE_H
