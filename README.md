# Unit tests for RapidSense and related perception components

To run integration and regtests (assumes megadeb or fully built rapidplan):
```
$ ./scripts/run_perc_tests.sh
```

If you are not running on a RTR controller, and you do not have the resources for the timing dependent tests. Run:
```
$ ./scripts/run_perc_tests.sh -d
```


## Intergration Tests
Currently for RapidSense and related perception components.

## Workspace Requirements
Move this repository into a catkin workspace. The folder structure should look like this:
```
.
└── catkin_ws
    └── src
        └── rapidsense_test
```
## Configuring the Workspace
When working in a new workspace, you will need to "extend" your host's ROS installation into the current workspace. The following command should be called from `catkin_ws/`
```
catkin config --extend /opt/ros/melodic
```

## Choosing a Rapidplan Source
#### [LOCAL BUILD]
If you plan on using a local rapidplan build, clone both `rapidsense_test` and `rapidplan` into a catkin workspace. The folder structure should look like this:
```
.
└── catkin_ws
    └── src
        ├── rapidplan
        └── rapidsense_test
```

#### [MEGADEB]
If you plan to use a megadeb with exposed libary header. Check if rapidplan-dev is currently installed.
```
apt policy rapidplan-dev
```

If the `rapidplan-dev` package is not installed, download and install from the build page https://build.realtime.cxm/job/bionic-melodic/job/megadeb/job/master/  (The debian file required is named `rapidplan-dev*deb.run`)
```
sudo chmod +x rapidplan-dev*deb.run && sudo ./rapidplan-dev*deb.run
```

## Build and run tests
To run tests, the tests packages must first be built.
```
catkin build && catkin run_tests
```

Once built, they can be run again using. When run this way, each node should log to the console.
```
rostest <path-to-launch-file>
```

To run a single test package, call the following
```
catkin run_tests <package_name>
catkin run_tests reg_test_calibration_sim
```

## Test Results
Tests results are organized by tests, and are stored in in the following folder
```
rapidsense_test/build/reg_test_calibration_sim/test_results/*
```

