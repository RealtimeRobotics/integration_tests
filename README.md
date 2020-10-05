# Unit tests for RapidSense and related perception components

To run integration and regtests (assumes megadeb or fully built rapidplan):
```
$ ./scripts/run_perc_tests.sh
```

If you are not running on a RTR controller, and you do not have the resources for the timing dependent tests. Run:
```
$ ./scripts/run_perc_tests.sh -d
```


# Intergration Tests
Currently for RapidSense and related perception components.

# Workspace Requirements
A megadeb with exposed libary headers must be installed. Check if it is currently installed...
```
apt policy rapidplan-dev
```

If the `rapidplan-dev` package is not installed, download and install from the build page https://build.realtime.cxm/job/bionic-melodic/job/megadeb/job/master/  (The debian file required is named `rapidplan-dev*deb.run`)
```
sudo chmod +x rapidplan-dev*deb.run && sudo ./rapidplan-dev*deb.run
```

# Configuring the Workspace
Every time you would like to work in the system, you need to "extend" your host's ROS installation into the current workspace...
```
catkin config --extend /opt/ros/melodic
```

# Build and run tests
To run tests, the tests packages must first be built.
```
catkin build && catkin run_tests
```

Once built, they can be run again using. When run this way, each node should log to the console.
```
rostest <path-to-launch-file>
```


# Test Results
Tests results are organized by tests, and are stored in in the following folder
```
rapidsense_test/build/reg_test_calibration_sim/test_results/*
```

