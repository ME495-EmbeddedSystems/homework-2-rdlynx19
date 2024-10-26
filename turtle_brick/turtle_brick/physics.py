"""Physics Module for the Brick."""
import math


class World:
    """Keep track of the physics of the world."""

    def __init__(self, brick, gravity, radius, dt):
        """
        Initialize the world.

        Args:
        ----
        brick: The (x,y,z) location of the brick
        gravity: the acceleration due to gravity in m/s^2
        radius: the radius of the platform
        dt: timestep in seconds of the physics simulation

        """
        self._brick = brick
        self.gravity = gravity
        self.platform_radius = radius
        self.time_step = dt

    @property
    def brick(self):
        """
        Get the brick's location.

        Return:
        ------
        (x,y,z) location of the brick

        """
        return self._brick

    @brick.setter
    def brick(self, location):
        """
        Set the brick's location.

        Args:
        ----
        location: the (x,y,z) location of the brick

        """
        self._brick[0] = location.x
        self._brick[1] = location.y
        self._brick[2] = location.z

    def drop(self):
        """Update the brick's location using gravity for one timestep."""
        self._brick[2] = self._brick[2] - (self.gravity*self.time_step)*(
            self.time_step)
        # pass

    def drop_brick_z(self, tilt_angle):
        """Update the brick's location by falling from the platform."""
        self._brick[0] = self._brick[0] - (self.gravity*(
            math.sin(tilt_angle))*self.time_step)*self.time_step
        self._brick[2] = self._brick[2] - (self.gravity*(
            math.cos(tilt_angle))*self.time_step)*self.time_step

    def drop_brick_x(self, tilt_angle):
        """Update the brick's x location when falling from the platform."""
        self._brick[0] = self._brick[0] - (self.gravity*(
            math.sin(tilt_angle))*self.time_step)*(self.time_step)
