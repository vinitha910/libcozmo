import math
import numpy as np

def create_choices(start, stop, num, include_zero):
    """
    Helper function to generate choices, generates [num] number of choices
    from [start] to [stop]

    Parameters
    ----------
    start : float
       the starting value of the sequence
    stop : float
	the end value of the sequence
    num : int
	number of samples to generate
    include_zero : bool
	True to add zero to choices, False to not
	This allows for 0 velocity in either the linear or angular direction

    Returns a list of choices
    """

    choices = np.linspace(start, stop, num)
    if include_zero:
        choices = np.unique(np.insert(choices, 0, 0, axis=0))
    return [truncate(val, 3) for val in list(choices)]


def truncate(number, digits) -> float:
    """
    Helper function to truncate floats to specified number of decimal places

    Parameters
    ----------
    number : float
	the number to truncate
    digits : int
	the number of decimal places to keep

    Returns the truncated number
    """
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper

