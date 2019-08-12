#! /usr/bin/env python3

import os
import unittest
import yaml
import pymongo as pm

from floor_detection.floor_detector import FloorDetector

class TestFloorDetection(unittest.TestCase):

    """ Unit Tests for floor detection common code"""

    @classmethod
    def setUpClass(cls):
        test_dir = os.path.abspath(os.path.dirname(__file__))
        main_dir = os.path.dirname(test_dir)
        cls.config_file_path = os.path.join(main_dir, 'config/floor_detection_config_brsu.yaml')
        cls.floor_measurement_map = {}
        with open(cls.config_file_path, 'r') as config_file:
            config = yaml.safe_load(config_file)
            for floor_data in config['floor_measurements']:
                cls.floor_measurement_map[floor_data['floor_number']] = floor_data['mean_pressure_diff']
            cls.reference_floor = config.get('reference_floor', 0)
            cls.pressure_diff_tolerance = config.get('pressure_diff_tolerance', 0.0)
            cls.redundant_measurement_count = config.get('redundant_measurement_count', 1)
            cls.filter_window_size = config.get('filter_window_size', 5)
        print(cls.reference_floor)
        print(cls.floor_measurement_map)
        print(cls.pressure_diff_tolerance)
        print(cls.redundant_measurement_count)
        print(cls.filter_window_size)

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.floor_detector = FloorDetector(
            reference_floor=self.reference_floor,
            floor_measurement_map=self.floor_measurement_map,
            pressure_diff_tolerance=self.pressure_diff_tolerance,
            redundant_measurement_count=self.redundant_measurement_count,
            filter_window_size=self.filter_window_size)

    def tearDown(self):
        del(self.floor_detector)

    def test_register_measurement_failure(self):
        success = self.floor_detector.register_measurements([])
        self.assertFalse(success)

    def test_register_measurement_success_append(self):
        measurements = [0.0 for _ in range(self.redundant_measurement_count)]
        success = self.floor_detector.register_measurements(measurements)
        self.assertTrue(success)
        self.assertEqual(len(self.floor_detector.measurements[0]), 1)

    def test_register_measurement_success_replace(self):
        for i in range(self.filter_window_size-1):
            measurements = [0.0 for _ in range(self.redundant_measurement_count)]
            success = self.floor_detector.register_measurements(measurements)
            self.assertTrue(success)
        measurements = [0.01 for _ in range(self.redundant_measurement_count)]
        success = self.floor_detector.register_measurements(measurements)
        self.assertTrue(success)
        self.assertEqual(len(self.floor_detector.measurements[0]), self.filter_window_size)
        print(self.floor_detector.measurements[0])
        self.assertSequenceEqual([measurement[-1] for measurement in self.floor_detector.measurements[0]], measurements)

if __name__ == '__main__':
    unittest.main()
