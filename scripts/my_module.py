import numpy as np


def _get_action(i):
    """Test action function for non-learning mode.

    Argument:
    i -- the state the agent is in
    """
    return np.array([1, 1] if i < 100 else [0, 0], dtype=int)
