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
    floor_measurement_ranges: Dict[int, Tuple[float]] -- a dictionary in which the keys
                              are floor numbers and the values are expected (min, max)
                              floor measurement ranges
    redundant_measurement_count: int -- number of pressure measurements per time unit
    filter_window_size: int -- window size used for median filtering (default 5)

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de
    '''
    def __init__(self, floor_measurement_ranges, redundant_measurement_count, filter_window_size=5):
        self.floor_measurement_ranges = dict(floor_measurement_ranges)
        self.redundant_measurement_count = redundant_measurement_count
        self.filter_window_size = filter_window_size
        self.measurements = [deque() for _ in range(self.redundant_measurement_count)]
        self.filtered_measurement_averages = np.zeros(self.redundant_measurement_count)

    def register_measurements(self, measurements):
        '''Stores the received measurements into a local buffer.

        Keyword arguments: List[float] -- a list of pressure measurements

        '''
        if len(measurements) != self.redundant_measurement_count:
            print('[floor_detector] Warning: Expected {0} measurements'.format(self.redundant_measurement_count))

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

    def determine_floor(self):
        '''Determines the current floor based on a set of measurements in a local
        buffer. Median filtering is used on the measurements and, if there are
        multiple redundant measurements, the filtered measurements are averaged
        before the floor is determined.

        Returns None if less than "self.filter_window_size" measurements have been received.

        '''
        if not self.__sufficient_measurements_received():
            print('[floor_detector] Error: Insufficient measurements received; expected {0} measurements for determining the floor'.format(self.filter_window_size))
            return None

        for i, measurements in enumerate(self.measurements):
            self.filtered_measurement_averages[i] = np.median(measurements)

        measurement_average = np.mean(self.filtered_measurement_averages)
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

    def __get_floor(self, measurement):
        '''Uses the information in "self.floor_measurement_ranges" for
        determining the floor corresponding to the given measurement.

        Keyword arguments:
        measurement: float -- a filtered and averaged pressure measurement

        '''
        current_floor = -1
        for floor, ranges in self.floor_measurement_ranges.items():
            if ranges[0] < measurement < ranges[1]:
                current_floor = floor
                break
        return current_floor
