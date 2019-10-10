import unittest
import proj1 as p1

class TestStringMethods(unittest.TestCase):

    def test_point_on_line(self):
        self.assertEqual(p1.point_on_line((1,1),((0,0),(2,2))), True)

    def test_point_on_line(self):
        self.assertEqual(p1.point_on_line((1,1),((0,1),(1,0))), False)

if __name__ == '__main__':
    unittest.main()