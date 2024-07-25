
from typing import Union

import numpy as np



class Vector:
    """
    Represents a vector in three-dimensional space.

    Wrapper around a numpy.ndarray

    Attributes:
        vec (numpy.ndarray): A numpy array representing the vector's coordinates in x, y, and z directions.

    Methods:
        __init__(self, x: float = 0, y: float = 0, z: float = 0): Initializes a Vector object with the given coordinates.
        x(self) -> float: Returns the x-coordinate value of the vector.
        y(self) -> float: Returns the y-coordinate value of the vector.
        z(self) -> float: Returns the z-coordinate value of the vector.
    """

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        """
        Initializes a Vector object with the given coordinates.

        Args:
            x (float): The x-coordinate value (default is 0).
            y (float): The y-coordinate value (default is 0).
            z (float): The z-coordinate value (default is 0).
        """

        self.vec : np.ndarray = np.array([x, y, z])

    def x(self) -> float:
        """
        Returns the x-coordinate value of the vector.

        Returns:
            float: The x-coordinate value.
        """
        return self.vec[0]

    def y(self) -> float:
        """
        Returns the y-coordinate value of the vector.

        Returns:
            float: The y-coordinate value.
        """
        return self.vec[1]

    def z(self) -> float:
        """
        Returns the z-coordinate value of the vector.

        Returns:
            float: The z-coordinate value.
        """
        return self.vec[2]





def clamp_mag(val: float, max_mag: float) -> float:
    """
    Clamps the value `val` between -max_mag and max_mag.

    Args:
        val (float): The value to be clamped.
        max_mag (float): The maximum magnitude allowed.

    Returns:
        float: The clamped value.

    """
    
    if val > max_mag:
        return max_mag

    if val < -max_mag:
        return -max_mag

    return val





def sign(val: Union[int, float]) -> int:
    """
    Returns the sign of a given value.

    Args:
        val (int or float): The value to determine the sign of.

    Returns:
        int: 1 if the value is positive, 0 if the value is zero, -1 if the value is negative.
    """
    if val > 0:
        return 1
    
    if val == 0:
        return 0
    
    return -1

