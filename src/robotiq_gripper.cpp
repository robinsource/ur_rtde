#include <ur_rtde/robotiq_gripper.h>

#include <boost/algorithm/clamp.hpp>
#include <boost/array.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/write.hpp>
#include <iostream>
#include <thread>

using boost::asio::ip::tcp;

namespace ur_rtde
{
using VariableDict = std::vector<std::pair<std::string, int>>;

/**
 * Split string into a vector of strings using the given delimiter
 */
static std::vector<std::string> split(const std::string& str, char delim = ' ')
{
  std::vector<std::string> Result;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delim))
  {
    Result.push_back(token);
  }
  return Result;
}

static bool isAck(const std::string& data)
{
  return data == "ack";
}

RobotiqGripper::RobotiqGripper(const std::string& Hostname, int Port, bool verbose)
    : hostname_(Hostname), port_(Port), verbose_(verbose), deadline_(io_service_)
{
  // No deadline is required until the first socket operation is started. We
  // set the deadline to positive infinity so that the actor takes no action
  // until a specific deadline is set.
  deadline_.expires_at(boost::posix_time::pos_infin);

  // Start the persistent actor that checks for deadline expiry.
  check_deadline();
}

void RobotiqGripper::connect(uint32_t timeout_ms)
{
  socket_.reset(new boost::asio::ip::tcp::socket(io_service_));
  socket_->open(boost::asio::ip::tcp::v4());
  boost::asio::ip::tcp::no_delay no_delay_option(true);
  boost::asio::socket_base::reuse_address sol_reuse_option(true);
  socket_->set_option(no_delay_option);
  socket_->set_option(sol_reuse_option);
  resolver_ = std::make_shared<tcp::resolver>(io_service_);
  tcp::resolver::query query(hostname_, std::to_string(port_));

  if (verbose_)
    std::cout << "Connecting..." << std::endl;
  deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
  boost::system::error_code ec_ = boost::asio::error::would_block;
  boost::asio::async_connect(*socket_, resolver_->resolve(query),
                             [&ec_](const boost::system::error_code& ec, const tcp::endpoint& endpoint) { ec_ = ec; });
  do
  {
    io_service_.run_one();
  } while (ec_ == boost::asio::error::would_block);
  if (ec_ || !socket_->is_open())
  {
    throw std::runtime_error("Timeout connecting to gripper device.");
  }
  conn_state_ = ConnectionState::CONNECTED;
  if (verbose_)
    std::cout << "Connected successfully to RobotIQ server: " << hostname_ << " at " << port_ << std::endl;
}

void RobotiqGripper::disconnect()
{
  /* We use reset() to safely close the socket,
   * see: https://stackoverflow.com/questions/3062803/how-do-i-cleanly-reconnect-a-boostsocket-following-a-disconnect
   */
  socket_.reset();
  conn_state_ = ConnectionState::DISCONNECTED;
  if (verbose_)
    std::cout << "RobotIQ - Socket disconnected" << std::endl;
}

bool RobotiqGripper::isConnected() const
{
  return conn_state_ == ConnectionState::CONNECTED;
}

std::string RobotiqGripper::receive()
{
  boost::array<char, 1024> recv_buffer_;
  boost::system::error_code error_;
  size_t buflen = socket_->read_some(boost::asio::buffer(recv_buffer_), error_);
  return std::string(recv_buffer_.elems, buflen);
}

void RobotiqGripper::send(const std::string& str)
{
  boost::asio::write(*socket_, boost::asio::buffer(str));
}

void RobotiqGripper::dumpVars()
{
  std::vector<std::string> vars = {"ACT", "GTO", "FOR", "SPE", "POS", "STA", "PRE", "OBJ", "FLT"};
  std::cout << "\nVariable dump: ---------------\n";
  for (auto const& var : vars)
  {
    std::cout << var << ": " << getVar(var) << std::endl;
  }
}

void RobotiqGripper::activate(bool auto_calibrate)
{
  if (!isActive())
  {
    if (verbose_)
      std::cout << "!Active" << std::endl;
    reset();
    while (getVar("ACT") != 0 || getVar("STA") != 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    setVar("ACT", 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while (getVar("ACT") != 1 || getVar("STA") != 3)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  if (verbose_)
    std::cout << "Active" << std::endl;
  if (auto_calibrate)
  {
    autoCalibrate();
  }
  dumpVars();
}

void RobotiqGripper::autoCalibrate()
{
  // first try to open in case we are holding an object
  auto status = move(getOpenPosition(), 64, 1, WAIT_FINISHED);
  if (status != AT_DEST)
  {
    throw std::runtime_error("Gripper calibration failed to start");
  }

  // try to close as far as possible, and record the number
  status = move(getClosedPosition(), 64, 1, WAIT_FINISHED);
  if (status != AT_DEST && status != STOPPED_INNER_OBJECT)
  {
    throw std::runtime_error("Gripper calibration failed");
  }
  max_position_ = std::min(getCurrentDevicePosition(), max_position_);
  if (STOPPED_INNER_OBJECT == status)
  {
    max_position_ -= 5;
  }
  max_position_ = std::min(max_position_, 255);

  // try to open as far as possible, and record the number
  status = move(getOpenPosition(), 64, 1, WAIT_FINISHED);
  if (status != AT_DEST && status != STOPPED_OUTER_OBJECT)
  {
    throw std::runtime_error("Gripper calibration failed");
  }
  if (STOPPED_OUTER_OBJECT == status)
  {
    min_position_ -= 5;
  }
  min_position_ = std::max(getCurrentDevicePosition(), min_position_);
  if (verbose_)
  {
    std::cout << "Gripper auto-calibrated to " << min_position_ << ", " << max_position_ << std::endl;
  }
}

int RobotiqGripper::getVar(const std::string& var)
{
  std::string cmd = "GET " + var + "\n";
  // atomic commands send/rcv
  std::string rx_string;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    send(cmd);
    rx_string = receive();
  }
  auto data = split(rx_string);
  if (data[0] != var)
  {
    throw std::logic_error("Unexpected response: data " + data[0] + " does not match " + var);
  }
  int value = std::stoi(data[1]);
  return value;
}

bool RobotiqGripper::setVars(const std::vector<std::pair<std::string, int>> Vars)
{
  std::string cmd = "SET";
  for (const auto& Var : Vars)
  {
    cmd += " " + Var.first + " " + std::to_string(Var.second);
  }
  cmd += "\n";
  // atomic commands send/rcv
  const std::lock_guard<std::mutex> lock(mutex_);
  send(cmd);
  auto data = receive();
  return isAck(data);
}

bool RobotiqGripper::setVar(const std::string& Var, int Value)
{
  return setVars({std::make_pair(Var, Value)});
}

bool RobotiqGripper::isActive()
{
  int status = getVar("STA");
  return ACTIVE == status;
}

float RobotiqGripper::getMinPosition() const
{
  return convertValueUnit((float)min_position_, POSITION, FROM_DEVICE_UNIT);
}

float RobotiqGripper::getMaxPosition() const
{
  return convertValueUnit((float)max_position_, POSITION, FROM_DEVICE_UNIT);
}

float RobotiqGripper::getOpenPosition() const
{
  return getMinPosition();
}

float RobotiqGripper::getClosedPosition() const
{
  return getMaxPosition();
}

int RobotiqGripper::getCurrentDevicePosition()
{
  return getVar("POS");
}

float RobotiqGripper::getCurrentPosition()
{
  return convertValueUnit((float)getCurrentDevicePosition(), POSITION, FROM_DEVICE_UNIT);
}

bool RobotiqGripper::isOpen()
{
  return getCurrentDevicePosition() == min_position_;
}

bool RobotiqGripper::isClosed()
{
  return getCurrentDevicePosition() == max_position_;
}

void RobotiqGripper::reset()
{
  setVar("ACT", 0);
  setVar("ATR", 0);
  while (getVar("ACT") != 0 || getVar("STA") != 0)
  {
    setVar("ACT", 0);
    setVar("ATR", 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void RobotiqGripper::emergencyRelease(ePostionId Direction, eMoveMode MoveMode)
{
  setVar("ATR", 0);
  setVar("ARD", (CLOSE == Direction) ? 0 : 1);
  setVar("ACT", 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  setVar("ATR", 1);

  // wait until the gripper acknowledges that it started auto release
  while (faultStatus() != FAULT_EMCY_RELEASE_ACTIVE)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (START_MOVE == MoveMode)
  {
    return;
  }

  // wait until the gripper finishes emergency release
  while (faultStatus() != FAULT_EMCY_RELEASE_FINISHED)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int RobotiqGripper::faultStatus()
{
  return getVar("FLT");
}

float RobotiqGripper::setSpeed(float Speed)
{
  int dev_speed = (int)convertValueUnit(Speed, SPEED, TO_DEVICE_UNIT);
  speed_ = boost::algorithm::clamp(dev_speed, min_speed_, max_speed_);
  return convertValueUnit((float)speed_, SPEED, FROM_DEVICE_UNIT);
}

float RobotiqGripper::setForce(float Force)
{
  int dev_force = (int)convertValueUnit(Force, FORCE, TO_DEVICE_UNIT);
  force_ = boost::algorithm::clamp(dev_force, min_force_, max_force_);
  return convertValueUnit((float)force_, FORCE, FROM_DEVICE_UNIT);
}

int RobotiqGripper::move(float fPosition, float fSpeed, float fForce, eMoveMode MoveMode)
{
  int Position = (int)convertValueUnit(fPosition, POSITION, TO_DEVICE_UNIT);
  int Speed = (int)convertValueUnit(fSpeed, SPEED, TO_DEVICE_UNIT);
  int Force = (int)convertValueUnit(fForce, FORCE, TO_DEVICE_UNIT);
  Speed = (fSpeed < 0) ? speed_ : Speed;
  Force = (fForce < 0) ? force_ : Force;
  Position = boost::algorithm::clamp(Position, min_position_, max_position_);
  Speed = boost::algorithm::clamp(Speed, min_speed_, max_speed_);
  Force = boost::algorithm::clamp(Force, min_force_, max_force_);
  VariableDict Vars{{"POS", Position}, {"SPE", Speed}, {"FOR", Force}, {"GTO", 1}};
  bool Result = setVars(Vars);
  if (!Result)
  {
    throw std::runtime_error("Failed to set variables for gripper move");
  }

  // wait until the gripper acknowledges that it will try to go to the requested position
  while (getVar("PRE") != Position)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (WAIT_FINISHED == MoveMode)
  {
    return waitForMotionComplete();
  }
  else
  {
    return objectDetectionStatus();
  }
}

int RobotiqGripper::open(float NormSpeed, float NormForce, eMoveMode MoveMode)
{
  return move(getOpenPosition(), NormSpeed, NormForce, MoveMode);
}

int RobotiqGripper::close(float NormSpeed, float NormForce, eMoveMode MoveMode)
{
  return move(getClosedPosition(), NormSpeed, NormForce, MoveMode);
}

RobotiqGripper::eObjectStatus RobotiqGripper::objectDetectionStatus()
{
  return (RobotiqGripper::eObjectStatus)getVar("OBJ");
}

RobotiqGripper::eObjectStatus RobotiqGripper::waitForMotionComplete()
{
  // wait until not moving
  int ObjectStatus = getVar("OBJ");
  while (MOVING == ObjectStatus)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ObjectStatus = getVar("OBJ");
  }

  return (RobotiqGripper::eObjectStatus)ObjectStatus;
}

float RobotiqGripper::convertValueUnit(float Value, eMoveParameter Param, eUnitConversion ConversionDirection) const
{
  auto Unit = units_[Param];
  if (UNIT_DEVICE == Unit)
  {
    return Value;
  }

  float factor = 1.0;
  float offset = 0.0;
  switch (Unit)
  {
    case UNIT_NORMALIZED:
      factor = 255.0;
      break;
    case UNIT_PERCENT:
      factor = float(1.0 / 100.0 * 255);
      break;
    case UNIT_MM:
      factor = float(1.0 / (max_position_mm_ - min_position_mm_) * 255);
      offset = float(min_position_mm_);
      break;
    default:
      break;
  }

  if (ConversionDirection == TO_DEVICE_UNIT)
  {
    int Result = int(roundf((Value - offset) * factor));
    Result = (POSITION == Param) ? (255 - Result) : Result;
    return float(Result);
  }
  else
  {
    Value = (POSITION == Param) ? (255 - Value) : Value;
    return (Value / factor) + offset;
  }
}

void RobotiqGripper::setUnit(eMoveParameter Param, eUnit Unit)
{
  units_[Param] = Unit;
}

void RobotiqGripper::setPositionRange_mm(int MinPosition, int MaxPosition)
{
  if (MinPosition > MaxPosition)
  {
    throw std::invalid_argument("MinPosition parameter is greater than MaxPosition parameter");
  }

  min_position_mm_ = MinPosition;
  max_position_mm_ = MaxPosition;
}

void RobotiqGripper::check_deadline()
{
  // Check whether the deadline has passed. We compare the deadline against
  // the current time since a new asynchronous operation may have moved the
  // deadline before this actor had a chance to run.
  if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {
    // The deadline has passed. The socket is closed so that any outstanding
    // asynchronous operations are cancelled. This allows the blocked
    // connect(), read_line() or write_line() functions to return.
    boost::system::error_code ignored_ec;
    socket_->close(ignored_ec);

    // There is no longer an active deadline. The expiry is set to positive
    // infinity so that the actor takes no action until a new deadline is set.
    deadline_.expires_at(boost::posix_time::pos_infin);
  }

  // Put the actor back to sleep.
  deadline_.async_wait(std::bind(&RobotiqGripper::check_deadline, this));
}

}  // namespace ur_rtde
