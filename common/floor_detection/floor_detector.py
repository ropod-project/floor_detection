import numpy as np
from collections import deque

class FloorDetector(object):
    '''Uses smart wheel pressure sensor measurements for determining the current floor
    of a robot. The floor is determined by checking whether the measurements are within given
    measurement limits; proper calibration of the pressure sensors is expected. The detector
    uses a median filter on a set of measurements and, if there are redundant
    pressure measurements, calculates an average of the measurements before
    determining the floor.

    Constructor keyword arguments:

    reference_floor: int -- floor with respect to which measurements pressure
                            measurements are expressed
    floor_measurement_map: Dict[int, float] -- a dictionary in which the keys
                           are floor numbers and the values are average
                           differences in pressure measurements with respect
                           to the reference floor
    pressure_diff_tolerance: float -- allowed deviation from the values in
                                      floor_measurement_map
    redundant_measurement_count: int -- number of pressure measurements per time unit
    filter_window_size: int -- window size used for median filtering (default 5)

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de
    '''
    def __init__(self, reference_floor, floor_measurement_map, pressure_diff_tolerance,
                 redundant_measurement_count, filter_window_size=5):
        self.reference_floor = reference_floor
        self.reference_floor_measurement_map = dict(floor_measurement_map)
        self.pressure_diff_tolerance = pressure_diff_tolerance
        self.redundant_measurement_count = redundant_measurement_count
        self.filter_window_size = filter_window_size
        self.measurements = [deque() for _ in range(self.redundant_measurement_count)]

        # a list of Booleans in which the i-th entry indicates whether
        # the i-th pressure measurement should be taken into account
        # for detecting floor changes (we should ideally ignore
        # faulty measurements)
        self.ignore_sensor_measurement = [False] * self.redundant_measurement_count

        self.floor_measurement_initialised = False
        self.current_floor = -1
        self.current_floor_measurement = 0.
        self.current_floor_measurement_map = dict(self.reference_floor_measurement_map)

    def set_current_floor(self, floor):
        '''Updates the information about the current floor.

        Keyword arguments:
        floor: int

        '''
        print('[floor_detector] Setting current floor: {0}'.format(floor))
        self.__update_floor(floor)

    def update_sensor_statuses(self, sensor_statuses):
        '''Updates the list of sensor statuses with the given input.

        Keyword arguments:
        sensor_statuses: List[Bool] -- a list of Booleans with as many entries
                                       as the number of redundant pressure sensors,
                                       where the i-th entry indicates whether
                                       the i-th sensor is operational

        '''
        if len(self.ignore_sensor_measurement) != len(sensor_statuses):
            print('[floor_detector, update_sensor_statuses] WARNING: Input list expected to have {0} entries; ignoring update'.format(sensor_statuses))
            return
        self.ignore_sensor_measurement = sensor_statuses

    def register_measurements(self, measurements):
        '''Stores the received measurements into a local buffer.

        Keyword arguments:
        measurements: List[float] -- a list of pressure measurements

        '''
        if len(measurements) != self.redundant_measurement_count:
            print('[floor_detector, register_measurements] WARNING: Expected {0} measurements; ignoring call'.format(self.redundant_measurement_count))
            return

        for i, measurement in enumerate(measurements):
            # if we haven't yet received enough measurements, we just
            # append the newest measurement to the measurement list
            if len(self.measurements[i]) < self.filter_window_size:
                self.measurements[i].append(measurement)
            # if we already have enough measurements, we remove the
            # oldest measurements and add the new one to the list
            else:
                self.measurements[i].popleft()
                self.measurements[i].append(measurement)

        if self.__sufficient_measurements_received() and not self.floor_measurement_initialised:
            self.current_floor_measurement = self.__average_measurements()
            self.floor_measurement_initialised = True

    def determine_floor(self):
        '''Determines the current floor based on a set of measurements in a local
        buffer. Median filtering is used on the measurements and, if there are
        multiple redundant measurements, the filtered measurements are averaged
        before the floor is determined.

        Returns None if less than "self.filter_window_size" measurements have been received.

        '''
        measurement_average = self.__average_measurements()
        return self.__get_floor(measurement_average)

    def __sufficient_measurements_received(self):
        '''Returns True if all entries in "self.measurements"
        have "self.filter_window_size" measurements; returns
        False otherwise.
        '''
        for measurements in self.measurements:
            if len(measurements) != self.filter_window_size:
                return False
        return True

    def __average_measurements(self):
        '''Returns an average of the received pressure measurements.
        The returned average is calculated using as

        \bar{measurements} = \frac{sum_{i=1}^{n}{median(self.measurements)}}{n}

        where n = self.redundant_measurement_count.

        Returns 0 if less than self.filter_window_size measurements have been received.
        '''
        if not self.__sufficient_measurements_received():
            return 0.

        filtered_measurement_averages = np.zeros(len(np.where(self.ignore_sensor_measurement)[0]))
        for i, measurements in enumerate(self.measurements):
            if not self.ignore_sensor_measurement[i]:
                filtered_measurement_averages[i] = np.median(measurements)

        measurement_average = np.mean(filtered_measurement_averages)
        return measurement_average

    def __get_floor(self, measurement):
        '''Uses the information in "self.current_floor_measurement_map" for
        determining the floor corresponding to the given measurement.

        Keyword arguments:
        measurement: float -- a filtered and averaged pressure measurement

        '''
        measurement_delta = measurement - self.current_floor_measurement
        for floor, reference_floor_delta in self.current_floor_measurement_map.items():
            ranges = (reference_floor_delta - self.pressure_diff_tolerance,
                      reference_floor_delta + self.pressure_diff_tolerance)
            if ranges[0] < measurement_delta < ranges[1]:
                if floor != self.current_floor:
                    self.__update_floor(floor)
                else:
                    self.current_floor_measurement = measurement
                break
        return self.current_floor

    def __update_floor(self, floor):
        '''Updates the information about the current floor (including the
        reference measurement for the current floor and the map of
        measurement deltas).

        Keyword arguments:
        floor: int

        '''
        self.current_floor = floor
        self.current_floor_measurement = self.__average_measurements()
        self.current_floor_measurement_map = self.__get_updated_floor_measurement_map()

    def __get_updated_floor_measurement_map(self):
        '''Returns a map with the same keys as self.reference_floor_measurement_map,
        such that the relative pressure differences between floors are calculated
        with respect to the current floor rather than the reference floor.
        '''
        floor_measurement_map = dict(self.reference_floor_measurement_map)
        current_floor_reference_delta = self.reference_floor_measurement_map[self.current_floor]
        for floor, reference_floor_delta in self.reference_floor_measurement_map.items():
            if floor == self.current_floor:
                floor_measurement_map[floor] = 0.
            else:
                floor_measurement_map[floor] = reference_floor_delta - current_floor_reference_delta
        return floor_measurement_map
