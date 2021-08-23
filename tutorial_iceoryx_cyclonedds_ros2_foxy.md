# Using CycloneDDS with shared memory support on ROS 2 Foxy

In this document is explained how to install, activate and use the shared memory support
on the CycloneDDS RMW implementation in ROS 2.

Recommended is to read additionally [the documentation on rmw_cyclonedds_cpp](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md).

## Prerequisites

You need to have an existing ROS 2 installation on your system via debian packages.

Please refer to this [documentation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) for installing the debian packages.

## Installation of CycloneDDS and iceoryx

Download all debian packages for iceoryx + cyclonedds into a custom folder.
Open a terminal in the folder and run for installing the packages:

```console
user@user /tmp/download_cyclonedds_iceoryx> sudo dpkg -i ros-foxy*.deb
```

## Example application with shm transport

To verify the functionality of iceoryx we reuse an existing example from ROS 2.
For that you can create a ROS 2 workspace and clone a prepared `demo` repository into the `src` folder:

```console
user@user ~/ros_foxy_ws/src> git clone https://github.com/ApexAI/demos.git -b foxy_iceoryx_cyclonedds_example
```

According to the current [limitations](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md#restrictions)
we can only used fixed-size data types (types that not allocate dynamically on the heap) to communicate over shared memory.

In the demo repository we created a talker/listener application that uses a "plain old datatype" (pod) for communicating over shared memory.
In our case is a `uint64` used. Bounded Arrays with fixed-size data types are allowed too.

Build the demo nodes with:

```console
user@user ~/ros_foxy_ws> source /opt/ros/foxy/setup.bash
user@user ~/ros_foxy_ws> colcon build
```

The shared memory transport is by default deactivated and needs to be enabled before running applications.
This is done by an xml config file. You can either create your own based on [this documentation](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md#configuration)
or reuse the existing `cyclonedds.xml` file in the root folder of the `demos` repository. We assume here that you prefer the latter.
The environment variable `CYCLONEDDS_URI` needs to point to path of the config file so that CycloneDDS reads it at startup.

For every new terminal window you need to re-enable the cyclonedds config.

As example application we are using the `talker_pod_counter` and `listener_pod_counter`.

Please create a new terminal window and enter the following:

```console
user@user ~/ros_foxy_ws> source /opt/ros/foxy/setup.bash
user@user ~/ros_foxy_ws> source install/setup.bash
user@user ~/ros_foxy_ws> export CYCLONEDDS_URI=file://$PWD/src/demos/cyclonedds.xml
user@user ~/ros_foxy_ws> RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp talker_pod_counter
```

Now create a second terminal and enter this:

```console
user@user ~/ros_foxy_ws> source /opt/ros/foxy/setup.bash
user@user ~/ros_foxy_ws> source install/setup.bash
user@user ~/ros_foxy_ws> export CYCLONEDDS_URI=file://$PWD/src/demos/cyclonedds.xml
user@user ~/ros_foxy_ws> RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp listener_pod_counter
```

***NOTE:*** In ROS 2 Foxy the default rmw implementation is Fast RTPS.
By setting the environment variable `RMW_IMPLEMENTATION` to `rmw_cyclonedds_cpp` you ensure that CycloneDDS with iceoryx is used.

You should now see a terminal output like this:

```console
2021-08-23 11:55:58.712 [Warning]: RouDi not found - waiting ...
```

Both applications are now waiting for a certain time that the iceoryx middleware daemon comes up.
RouDi (Routing and Discovery) is a separate application that we start in a new terminal window:

```console
user@user ~/ros_foxy_ws> source /opt/ros/foxy/setup.bash
user@user ~/ros_foxy_ws> iox-roudi
```

Please note that you need to start always `iox-roudi` in parallel when you want to use shared memory over iceoryx.

The applications should now begin to run and you should see transferred data on the listener terminal:

```console
user@user ~/ros_foxy_ws> ...I heard: [Hello World: 5]
```

With that you have a successfull data transfer over CycloneDDS.

## Test if iceoryx shared memory transport is used

For additional debugging, iceoryx offers an introspection client that tell you some insights about the middleware.
You can use that tool to verify that iceoryx is used, because ROS 2 silently falls back to DDS communication
if shared memory communication is not possible (e.g. when dynamic data types like a `std::string` is used).

As preparation you need to clone this repo into you workspace:

```console
user@user ~/ros_foxy_ws> git clone https://github.com/eclipse-iceoryx/iceoryx.git src/eclipse-iceoryx/iceoryx -b v1.0.1
user@user ~/ros_foxy_ws> sudo apt install libacl1-dev libncurses5-dev cmake gcc g++
```

You can now take a look to [this documentation](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md#verifing-shared-memory-usage)
to build and run the introspection in parallel to the example apps. The interpretation of the data you see is here explained too.

Please note that you need to source the ros foxy bash again before building the introspection, otherwise CMake cannot find iceoryx.
