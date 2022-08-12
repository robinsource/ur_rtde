******
Guides
******
This section contains guides for how to use the ur_rtde interface.

.. _realtime-setup-guide:

Real-time Setup Guide
=====================
ur_rtde makes it possible to specify a desired real-time priority for the RTDEControlInterface and RTDEReceiveInterface.
First you must make sure your OS supports a real-time kernel and install it, if not already available. In the following
sections you can find how to set it up on various commonly used operating systems.

Linux
-----
The linux kernel is not real-time capable by default, and therefore one has to be installed.

Ubuntu 22.04
~~~~~~~~~~~~
For Ubuntu 22.04 a real-time beta kernel is already installed, but it has to be enabled. See more
information here: `Real-time Ubuntu 22.04 LTS Beta <https://ubuntu.com/blog/real-time-ubuntu-released>`_

How to enable the real-time kernel beta
"""""""""""""""""""""""""""""""""""""""
The beta kernel is available for free for personal use via Ubuntu Advantage for Infrastructure (UA-I)
To attach your personal machine to a UA subscription, please run:

.. code-block:: shell

    ua attach <free token>

where the <free token> is found on your Ubuntu one account associated with the free UA subscription.

Make sure you have at least version 27.8 of ubuntu-advantage-tools. You can check the version with:

.. code-block:: shell

    ua version

To upgrade ubuntu-advantage-tools to 27.8 in Jammy Jellyfish, please run:

.. code-block:: shell

    sudo apt install ubuntu-advantage-tools=27.8~22.04.1

To enable the real-time beta kernel, run:

.. code-block:: shell

    ua enable realtime-kernel --beta

Now reboot the system and the realtime kernel should be loaded.

.. warning::
    Please note you will need to manually configure grub to revert back to your original kernel after enabling real-time.

    For more information: ua help realtime-kernel

Ubuntu 20.04 / 18.04 + Other debian distros
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To get real-time support into a ubuntu system, the following steps have to be performed:

1. Get the sources of a real-time kernel
2. Compile the real-time kernel
3. Install real-time kernel

Install dependencies
""""""""""""""""""""
To build the kernel, you will need a couple of tools available on your system. You can install them using

.. code-block:: shell

   sudo apt-get install build-essential bc ca-certificates gnupg2 libssl-dev wget gawk flex bison

Before you download the sources of a real-time-enabled kernel, check the kernel version that is currently installed:

.. code-block:: shell

   uname -r
   4.15.0-62-generic

To continue with this tutorial, please create a temporary folder and navigate into it.
You should have sufficient space (around 25GB) there, as the extracted kernel sources take much space.
After the new kernel is installed, you can delete this folder again.

In this example we will use a temporary folder inside our home folder:

.. code-block:: shell

   mkdir -p ${HOME}/rt_kernel_build
   cd ${HOME}/rt_kernel_build

Getting the sources for a real-time kernel
""""""""""""""""""""""""""""""""""""""""""
To build a real-time kernel, we first need to get the kernel sources and the real-time patch.

First, we must decide on the kernel version that we want to use. Above, we determined that our system has a 4.15 kernel
installed. However, real-time patches exist only for selected kernel versions. Those can be found on the
linuxfoundation wiki.

In this example, we will select a 4.14 kernel. Select a kernel version close to the one installed on your system.

Go ahead and download the kernel sources, patch sources and their signature files:

.. code-block:: shell

   wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/4.14/patch-4.14.139-rt66.patch.xz
   wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/4.14/patch-4.14.139-rt66.patch.sign
   wget https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.139.tar.xz
   wget https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.139.tar.sign

To unzip the downloaded files do

.. code-block:: shell

   xz -dk patch-4.14.139-rt66.patch.xz
   xz -d linux-4.14.139.tar.xz

Verification
""""""""""""
Technically, you can skip this section, it is however highly recommended to verify the file integrity of such a core
component of your system!

To verify file integrity, you must first import public keys by the kernel developers and the patch author.
For the kernel sources use (as suggested on kernel.org)

.. code-block:: shell

   gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org

and for the patch search for a key of the author listed on linuxfoundation wiki.

.. code-block:: shell

    gpg2 --keyserver hkp://keys.gnupg.net --search-keys zanussi
    gpg: data source: http://51.38.91.189:11371
    (1)     German Daniel Zanussi <german.zanussi@globant.com>
              4096 bit RSA key 0x537F98A9D92CEAC8, created: 2019-07-24, expires: 2023-07-24
    (2)     Michael Zanussi <mzanussi@gmail.com>
              4096 bit RSA key 0x7C7F76A2C1E3D9EB, created: 2019-05-08
    (3)     Tom Zanussi <tzanussi@gmail.com>
            Tom Zanussi <zanussi@kernel.org>
            Tom Zanussi <tom.zanussi@linux.intel.com>
              4096 bit RSA key 0xDE09826778A38521, created: 2017-12-15
    (4)     Riccardo Zanussi <riccardo.zanussi@gmail.com>
              2048 bit RSA key 0xD299A06261D919C3, created: 2014-08-27, expires: 2018-08-27 (expired)
    (5)     Zanussi Gianni <g.zanussi@virgilio.it>
              1024 bit DSA key 0x78B89CB020D1836C, created: 2004-04-06
    (6)     Michael Zanussi <zanussi@unm.edu>
            Michael Zanussi <mzanussi@gmail.com>
            Michael Zanussi <michael_zanussi@yahoo.com>
            Michael Zanussi <michael@michaelzanussi.com>
              1024 bit DSA key 0xB3E952DCAC653064, created: 2000-09-05
    (7)     Michael Zanussi <surfpnk@yahoo.com>
              1024 bit DSA key 0xEB10BBD9BA749318, created: 1999-05-31
    (8)     Michael B. Zanussi <surfpnk@yahoo.com>
              1024 bit DSA key 0x39EE4EAD7BBB1E43, created: 1998-07-16
    Keys 1-8 of 8 for "zanussi".  Enter number(s), N)ext, or Q)uit > 3

Compilation
"""""""""""
Before we can compile the sources, we have to extract the tar archive and apply the patch

.. code-block:: shell

    tar xf linux-4.14.139.tar
    cd linux-4.14.139
    xzcat ../patch-4.14.139-rt66.patch.xz | patch -p1

Now to configure your kernel, just type

.. code-block:: shell

    make oldconfig

This will ask for kernel options. For everything else then the Preemption Model use the default value (just press Enter)
or adapt to your preferences. For the preemption model select Fully Preemptible Kernel:

.. code-block:: shell

   Preemption Model
      1. No Forced Preemption (Server) (PREEMPT_NONE)
    > 2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
      3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT__LL) (NEW)
      4. Preemptible Kernel (Basic RT) (PREEMPT_RTB) (NEW)
      5. Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)
    choice[1-5]: 5

Now you can build the kernel. This will take some time...

.. code-block:: shell

    make -j `getconf _NPROCESSORS_ONLN` deb-pkg

Installation
""""""""""""

After building, install the linux-headers and linux-image packages in the parent folder
(only the ones without the -dbg in the name)

.. code-block:: shell

    sudo apt install ../linux-headers-4.14.139-rt66_*.deb ../linux-image-4.14.139-rt66_*.deb



.. note::
   NVIDIA Drivers are not supported on PREEMPT_RT kernels!

Setup user privileges to use real-time scheduling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To be able to set real-time priority on threads with user privileges you'll have to change the user's limits by
changing :file:`/etc/security/limits.conf` (See the manpage for details).

It is recommended to setup a group for real-time users instead of writing a fixed username into the config file:

.. code-block:: shell

    sudo groupadd realtime
    sudo usermod -aG realtime $(whoami)

Afterwards, add the following limits to the *realtime* group in :file:`/etc/security/limits.conf` contains:

.. code-block:: shell

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400

You need to log out and in again or simply reboot in order for the new limits to take effect.

Windows
-------
The Windows NT kernel is real-time capable by default, this means we do not need to setup anything special to
execute ur_rtde with real-time priority, although you might have to run your program as Administrator.
On Windows the real-time priorities are set differently than on Linux and in another range.

Learn more about the real-time priorities on windows here: `Scheduling Priorities <https://docs.microsoft.com/en-us/windows/win32/procthread/scheduling-priorities>`_

Setting a real-time priority
---------------------------
The real-time priority of the RTDEControl and RTDEReceiveInterface can be set with an integer specified
in the constructor of the interfaces. Like so:

.. code-block:: c++

    // ur_rtde real-time priorities
    int rt_receive_priority = 90;
    int rt_control_priority = 85;

    RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
    RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);

    // Set application real-time priority
    RTDEUtility::setRealtimePriority(80);

On linux the priority range is (0-99) where 99 is the highest priority available. However be aware that a priority
of 99 might make the OS unstable. If no priority is specified the interface will default to the safe maximum priority
of 90. If a negative priority is specified, real-time priority will be disabled.

On Windows the REALTIME_PRIORITY_CLASS is set for the process, this means that the priority range is (16-31), where
the priority levels are:

+-------------------------------+---------------+
| Thread priority level         | Base priority |
+===============================+===============+
| THREAD_PRIORITY_IDLE          | 16            |
+-------------------------------+---------------+
| THREAD_PRIORITY_LOWEST        | 22            |
+-------------------------------+---------------+
| THREAD_PRIORITY_BELOW_NORMAL  | 23            |
+-------------------------------+---------------+
| THREAD_PRIORITY_NORMAL        | 24            |
+-------------------------------+---------------+
| THREAD_PRIORITY_ABOVE_NORMAL  | 25            |
+-------------------------------+---------------+
| THREAD_PRIORITY_HIGHEST       | 26            |
+-------------------------------+---------------+
| THREAD_PRIORITY_TIME_CRITICAL | 31            |
+-------------------------------+---------------+

Also see the more complete real-time control example under :file:`examples/cpp/realtime_control_example.cpp`

.. code-block:: c++

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

    std::vector<double> getCircleTarget(const std::vector<double> &pose, double timestep, double radius=0.075, double freq=1.0)
    {
      std::vector<double> circ_target = pose;
      circ_target[0] = pose[0] + radius * cos((2 * M_PI * freq * timestep));
      circ_target[1] = pose[1] + radius * sin((2 * M_PI * freq * timestep));
      return circ_target;
    }

    int main(int argc, char* argv[])
    {
      // Setup parameters
      std::string robot_ip = "localhost";
      double rtde_frequency = 500.0; // Hz
      double dt = 1.0 / rtde_frequency; // 2ms
      uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
      int ur_cap_port = 50002;

      // ur_rtde realtime priorities
      int rt_receive_priority = 90;
      int rt_control_priority = 85;

      RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
      RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);

      // Set application realtime priority
      RTDEUtility::setRealtimePriority(80);

      // Move parameters
      double vel = 0.5;
      double acc = 0.5;

      // Servo control parameters
      double lookahead_time = 0.1;
      double gain = 600;

      signal(SIGINT, raiseFlag);

      double time_counter = 0.0;

      // Move to init position using moveL
      std::vector<double> actual_tcp_pose = rtde_receive.getActualTCPPose();
      std::vector<double> init_pose = getCircleTarget(actual_tcp_pose, time_counter);
      rtde_control.moveL(init_pose, vel, acc);

      try
      {
        while (running)
        {
          rtde_control.initPeriod();
          std::vector<double> servo_target = getCircleTarget(actual_tcp_pose, time_counter);
          rtde_control.servoL(servo_target, vel, acc, dt, lookahead_time, gain);
          rtde_control.waitPeriod(dt);
          time_counter += dt;
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


Use with Dockerized UR Simulator
================================
See (https://github.com/urrsk/ursim_docker/blob/main/README.md for details)

first you need to clone the ursim_docker repository with:

.. code-block:: shell

    git clone https://github.com/urrsk/ursim_docker.git


Install docker
--------------
Next we install docker:

.. code-block:: shell

    sudo apt update
    sudo apt install docker.io
    sudo systemctl start docker
    sudo systemctl enable docker
    sudo systemctl status docker
    sudo usermod -aG docker $USER
    su - $USER


Build docker image
------------------
Then we build the docker image:

.. code-block:: shell

    docker build ursim/e-series -t myursim --build-arg VERSION=5.11.1.108318 --build-arg URSIM="https://s3-eu-west-1.amazonaws.com/ur-support-site/118926/URSim_Linux-5.11.1.108318.tar.gz"


Run docker image
----------------
Finally we run the docker image with:

.. code-block:: shell

     docker run --rm -it -p 5900:5900 -p 29999:29999 -p 30001-30004:30001-30004 myursim

.. _use-with-matlab:

Use with MATLAB
===============
MATLAB supports calling python library functions, please see
`this <https://se.mathworks.com/help/matlab/getting-started-with-python.html>`_ site for more information.

Here is an example of receiving the actual joint and tcp pose from the robot, and moving the robot
to some pre-defined cartesian position in MATLAB:

.. code-block:: matlab

    import py.rtde_receive.RTDEReceiveInterface
    import py.rtde_control.RTDEControlInterface

    rtde_r = RTDEReceiveInterface("localhost");
    rtde_c = RTDEControlInterface("localhost");

    actual_q = rtde_r.getActualQ();
    actual_tcp_pose = rtde_r.getActualTCPPose();

    % Convert to MATLAB array of double
    actual_q_array = cellfun(@double, cell(actual_q));
    actual_tcp_pose_array = cellfun(@double, cell(actual_tcp_pose));

    actual_q_array
    actual_tcp_pose_array

    position1 = [-0.343, -0.435, 0.50, -0.001, 3.12, 0.04];
    position2 = [-0.243, -0.335, 0.20, -0.001, 3.12, 0.04];

    rtde_c.moveL(position1);
    rtde_c.moveL(position2);
    rtde_c.stopRobot();
    clear

.. warning::
    Please notice, it is very important to include the 'clear' command and the end of execution, otherwise the ur_rtde
    threads will continue run in the background and you would not be able to execute the code again until the environment
    has been cleared.

.. note::
    Currently using the ur_rtde interface has only been tested with MATLAB R2019b using Python 2.7, since this seems
    to be the default interpreter of MATLAB R2019b. However, it should also work with Python 3.x


.. _use-with-robotiq-gripper:

Use with Robotiq Gripper
========================
There are currently 3 ways of using a Robotiq gripper with ur_rtde:

* **Option 1**: (Sending the robotiq preamble + function to be executed)

You can send the robotiq preamble script together with the function you want to run, using the
sendCustomScriptFunction() of the rtde_control interface. Unfortunately you have to send the preamble with
the gripper api functions everytime, which does give a bit of delay. You can download the preamble for
use with Python here: `robotiq_preamble.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_preamble.py>`_,
and a python interface for using the robotiq gripper this way here:
`robotiq_gripper_control.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_gripper_control.py>`_.

Example of this method:

.. code-block:: python

    from robotiq_gripper_control import RobotiqGripper
    from rtde_control import RTDEControlInterface
    import time

    rtde_c = RTDEControlInterface("<ROBOT_IP>")
    gripper = RobotiqGripper(rtde_c)

    # Activate the gripper and initialize force and speed
    gripper.activate()  # returns to previous position after activation
    gripper.set_force(50)  # from 0 to 100 %
    gripper.set_speed(100)  # from 0 to 100 %

    # Perform some gripper actions
    gripper.open()
    gripper.close()
    time.sleep(1)
    gripper.open()
    gripper.move(10)  # mm

    # Stop the rtde control script
    rtde_c.stopRobot()

.. admonition:: Pros
  :class: tip

    * Does not require any UR Cap to be installed.

.. admonition:: Cons
  :class: error

    * Slow execution, since the preamble is transmitted each time.
    * Simultaneous robot movements is not possible (since the rtde_control script is interrupted)

* **Option 2**: (Using the RS485 UR Cap)

Download the RS485 UR cap from here
`rs485-1.0.urcap <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/raw/master/ur_robot_driver/resources/rs485-1.0.urcap>`_,
install it on the robot and remember to remove the Robotiq_Grippers UR Cap as
these two cannot function together. It does not work with the Robotiq_Grippers UR Cap since this cap occupies the
RS485 port all the time.

You can then use the tool_communication script for making the robotiq serial port
available on your desktop. (eg. /tmp/ttyUR). Finally use a modbus RTU based driver to communicate through the serial
port. Alternatively you can avoid running the tool_communication script and just communicate directly to the socket at
the port specified in the RS485 cap (default is *54321*).

.. admonition:: Pros
  :class: tip

    * Allows you to communicate to the RS485 port on the robot.
    * This approach can be used with different grippers, that uses the UR RS485 connection.
    * Fast communication.

.. admonition:: Cons
  :class: error

    * Does not work together with the official Robotiq_Grippers UR Cap.
    * Requires you to install a UR Cap.

* **Option 3**: (Communicating directly with Robotiq_grippers UR Cap port)

A robotiq gripper can be controlled through a port (*63352*) that is opened by the Robotiq_grippers UR Cap. This
port provides direct communication to the gripper. So you simply connect to the robot IP at this port and you
can command it using the Robotiq string commands, see the 'Control' section of this
`manual <https://assets.robotiq.com/website-assets/support_documents/document/Hand-E_Manual_UniversalRobots_PDF_20191219.pdf>`_.

*C++*:

ur_rtde includes a C++ interface for robotiq grippers implemented by (Uwe Kindler). See the API here:
:ref:`Robotiq Gripper API <robotiq-gripper-api>`, and the example here: :ref:`Robotiq Gripper Example <robotiq-gripper-example>`

*Python*:

You can download an example Python class for controlling the gripper using this method here: `robotiq_gripper.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_gripper.py>`_.
This class was implemented by Sam (Rasp) thanks! The class can be used like this:

.. code-block:: python

    import robotiq_gripper
    import time

    ip = "127.0.0.1"

    def log_info(gripper):
        print(f"Pos: {str(gripper.get_current_position()): >3}  "
              f"Open: {gripper.is_open(): <2}  "
              f"Closed: {gripper.is_closed(): <2}  ")

    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(ip, 63352)
    print("Activating gripper...")
    gripper.activate()

    print("Testing gripper...")
    gripper.move_and_wait_for_pos(255, 255, 255)
    log_info(gripper)
    gripper.move_and_wait_for_pos(0, 255, 255)
    log_info(gripper)


.. admonition:: Pros
  :class: tip

    * Works with Robotiq_grippers UR Cap.
    * Fast communication.

.. admonition:: Cons
  :class: error

    * You might not be able to leverage existing robotiq drivers, depending on implementation.

My current recommendation is to use **Option 3** for controlling a Robotiq gripper, and if that does not suit your needs
go with **Option 2**. **Option 1** should only be used as a last resort.
