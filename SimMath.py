
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



    def x(self, set: float | int | None = None) -> float:
        """
        Returns the x-coordinate value of the vector.

        Returns:
            float: The x-coordinate value.
        """

        if set is not None:
            self.vec[0] = set

        return self.vec[0]



    def y(self, set: float | int | None = None) -> float:
        """
        Returns the y-coordinate value of the vector.

        Returns:
            float: The y-coordinate value.
        """

        if set is not None:
            self.vec[1] = set

        return self.vec[1]



    def z(self, set: float | int | None = None) -> float:
        """
        Returns the z-coordinate value of the vector.

        Returns:
            float: The z-coordinate value.
        """

        if set is not None:
            self.vec[2] = set

        return self.vec[2]
    

    def magnitude(self) -> float:
        return float(np.linalg.norm(self.vec))

    

    def normalized(self) -> 'Vector':
        """
        Returns a normalized version of the vector.

        Returns:
            Vector: A new Vector instance with normalized coordinates.
        """

        norm = np.linalg.norm(self.vec)

        if norm == 0:
            return Vector(0, 0, 0)
        
        normalized_vec = self.vec / norm

        return Vector(normalized_vec[0], normalized_vec[1], normalized_vec[2])
    


    def dot(self, other: 'Vector') -> float:
        return np.dot(self.vec, other.vec)
    


    def __add__(self, other: 'Vector') -> 'Vector':
        """
        Performs component-wise addition with another vector.

        Args:
            other (Vector): Another vector to perform addition with.

        Returns:
            Vector: A new Vector instance representing the result of the addition.
        """

        added_vec = self.vec + other.vec
        return Vector(added_vec[0], added_vec[1], added_vec[2])
    



    def __mul__(self, scalar: int | float) -> 'Vector':
        """
        Multiply the vector by a scalar or perform dot product with another vector.

        Parameters:
            scalar (int | float | None): The scalar value to multiply the vector by. Defaults to None.
            other (Vector | None): Another vector to perform dot product with. Defaults to None.

        Returns:
            Vector | float | None: If scalar is provided, returns a new Vector object representing the result of the multiplication.
            If other is provided, returns the dot product of the two vectors as a float.
            If neither scalar nor other is provided, returns None.
        """

        multiplied_vec = self.vec * scalar
        return Vector(multiplied_vec[0], multiplied_vec[1], multiplied_vec[2])
    



    def __truediv__(self, scalar: int | float) -> 'Vector':
        """
        Performs scalar division on the vector.

        Args:
            scalar (int or float): The scalar value to divide the vector by.

        Returns:
            Vector: A new Vector instance representing the result of the scalar division.
        """

        divided_vec = self.vec / scalar

        return Vector(divided_vec[0], divided_vec[1], divided_vec[2])
    


    def __neg__(self) -> 'Vector':
        """
        Performs unary negation on the vector.

        Returns:
            Vector: A new Vector instance representing the negation of the vector.
        """

        negated_vec = -self.vec

        return Vector(negated_vec[0], negated_vec[1], negated_vec[2])






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





def sign(val: int | float) -> int:
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

