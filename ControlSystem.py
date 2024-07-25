
"""
This is the Glider's control system module.

It provides a PID controller class, a state machine class, and logging.
"""




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





class PIDController:
    """
    Simple PID controller implementation.

    Includes derivative kickback, integral windup, and output protections.
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0, i_limit: float = 1_000.0, output_limit: float = 1_000.0) -> None:
        """
        Initialize the PID controller.

        Args:
            kp (float): Proportional gain (default: 1.0).
            ki (float): Integral gain (default: 0.0).
            kd (float): Derivative gain (default: 0.0).
            i_limit (float): Integral limit (default: 1_000.0).
            output_limit (float): Output limit (default: 1_000.0).
        """

        # Tuning parameters
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        # Integral windup limit
        self.i_limit = i_limit

        # Output limit
        self.output_limit = output_limit

        # Previous values
        self.prev_error = 0
        self.prev_input = 0
        self.prev_time = 0

        # Integral term
        self.integral = 0



    def update(self, target, input, time):
        """
        Updates the control system based on the target, input, and current time.

        Args:
            target (float): The desired target value.
            input (float): The current input value.
            time (float): The current time.

        Returns:
            float: The output value of the control system.

        Raises:
            ValueError: If the current time is not after the previous time.
        """

        # Don't return anything on first call
        if time == 0:
            self.prev_error = error
            self.prev_time = time
            self.prev_input = input
            return 0

        time_delta = time - self.prev_time

        if time_delta <= 0:
            raise ValueError("time must be larger than prev time")
        
        error = target - input

        # Integral windup prevention
        self.integral = clamp_mag(self.integral + (error * time_delta), self.i_limit)

        # Derivative kickback prevention
        derivative = (input - self.prev_input) / time_delta

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Store previous values
        self.prev_error = error
        self.prev_time = time
        self.prev_input = input

        return clamp_mag(output, self.output_limit)






