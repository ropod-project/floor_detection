# floor_detection

## Summary

A component (wrapped in a ROS node) detecting floor changes using pressure sensor measurements from smart wheels. The floor is determined by checking whether the measurements are within predefined measurement limits corresponding to different floors in a building.

The node exposes a service (`floor_detection_server` by default, but the name can be changed in the component launch file). Since only changes in floor are detected rather than an absolute level, the floor needs to be initialised manually (or detected in a different manner) for the calculations to be of any meaningful value. Initialisation is done by publishing to a topic (`/set_floor` by default; the name can be changed in the launcher).

For a more formal description of the component, please look at [this Jupyter notebook](docs/formal_description.ipynb).

## Usage

1. Launch the detector node:
```
roslaunch floor_detection floor_detection.launch
```
2. Initialise the current floor, for example through the command line:
```
rostopic pub /set_floor std_msgs/Int32 "data: 0"
```
3. Call the service to detect the current floor, for example through the command line:
```
rosservice call /floor_detection_server "{}"
```

For the component to work, make sure that data from the smart wheels is being published continuously.

### Launch file parameters

The following parameters can be passed to the detector node:
* `config_file_path`: Absolute path to a configuration file for the floor detector
* `server_name`: Name of the floor detection service exposed by the node (default `floor_detection_server`)
* `set_floor_topic`: Name of a topic on which the floor can be set externally (default `/set_floor`)
* `sw_data_topic`: Name of a topic on which smart wheel data are published (default `/sw_ethercat_parser/data`)

## Configuration

The detector uses the following configuration parameters:
* a reference floor
* number of redundant measurements
* window size specifying the number of most recent measurements to be kept at any point in time (used for noise filtering)
* expected pressure differences between floors expressed with respect to the reference floor
* error tolerance for the pressure differences

The configuration parameters need to be specified in a YAML file that has the following format:

```
reference_floor: int
floor_measurements:
- floor_number: int
  mean_pressure_diff: float
- floor_number: int
  mean_pressure_diff: float
pressure_diff_tolerance: float
redundant_measurement_count: int
filter_window_size: int
```

## Altitude pressure difference calibration

For calibrating the pressure differences between floors:
1. Choose a reference floor (e.g. the ground floor of a building)
2. Collect pressure data for each floor (since pressure varies throughout the day, the data collection should ideally be performed in a short period of time, e.g. within half an hour)
3. Calculate the average pressure differences between floors

## Requirements

* `numpy`
