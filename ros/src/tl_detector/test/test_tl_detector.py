#!/usr/bin/python
PKG = 'tl_detector'
NAME = 'test_tl_detector'

import sys
import time
import unittest
import rospy


class TestTLDetector(unittest.TestCase):
    def __init__(self, *args):
        super(TestTLDetector, self).__init__(*args)

    def setUp(self):
        rospy.init_node(NAME)

    def test_simple(self):
        pass


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestTLDetector)
