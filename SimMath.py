
from typing import Union



class vector:
    """
    Represents a vector in three-dimensional space.

    Attributes:
        x (float): The x-coordinate of the vector.
        y (float): The y-coordinate of the vector.
        z (float): The z-coordinate of the vector.
    """

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        """
        Initializes an instance of the ClassName class.

        Args:
            x (float): The x-coordinate value (default is 0).
            y (float): The y-coordinate value (default is 0).
            z (float): The z-coordinate value (default is 0).
        """

        self.x = x
        self.y = y
        self.z = z





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

