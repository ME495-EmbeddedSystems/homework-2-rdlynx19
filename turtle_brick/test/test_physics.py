"""Unittest for the physics module functions."""
import unittest

from geometry_msgs.msg import Point

from turtle_brick.physics import World


class TestPhysicsFunctions(unittest.TestCase):
    """Testing the physics.World member functions."""

    def setUp(self):
        """Create an instance of physics.World for all test methods."""
        self.test_brick_location = [6.0, 6.0, 8.0]
        self.test_world = World(self.test_brick_location, 9.8, 0.3, 0.004)

    def test_init(self):
        """Test the init function of physics.World."""
        self.assertEqual(self.test_world.brick, self.test_brick_location)
        self.assertEqual(self.test_world.gravity, 9.8)
        self.assertEqual(self.test_world.time_step, 0.004)
        self.assertEqual(self.test_world.platform_radius, 0.3)

    def test_brick_location_getter(self):
        """Test that the brick location is returned correctly."""
        response_brick_location = self.test_world.brick
        self.assertEqual(self.test_brick_location, response_brick_location)

    def test_brick_location_setter(self):
        """Test that the brick location is set correctly."""
        new_brick_location = Point()
        new_brick_location.x = 5.0
        new_brick_location.y = 5.0
        new_brick_location.z = 5.0
        lst_brick_location = [new_brick_location.x,
                              new_brick_location.y,
                              new_brick_location.z]
        self.test_world.brick = new_brick_location
        updated_brick_location = self.test_world.brick
        self.assertEqual(lst_brick_location, updated_brick_location)

    def test_brick_drop(self):
        """Test that the brick location is updated correctly."""
        brick_vel = 0.0
        brick_vel += 9.8 * 0.004
        new_brick_z = self.test_brick_location[2] - (brick_vel)*(0.004)
        updated_brick_location = [
            self.test_brick_location[0],
            self.test_brick_location[1],
            new_brick_z]
        self.test_world.drop()
        self.assertEqual(updated_brick_location, self.test_world.brick)


if __name__ == '__main__':
    unittest.main()
