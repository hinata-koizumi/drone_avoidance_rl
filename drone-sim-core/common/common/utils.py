"""Utility functions for drone RL (typed for mypy, PEP 561)."""

def clamp(v: float, lo: float, hi: float) -> float:
    """
    Clamp a value v between lower (lo) and upper (hi) bounds.
    Args:
        v (float): Value to clamp.
        lo (float): Lower bound.
        hi (float): Upper bound.
    Returns:
        float: Clamped value.
    """
    return float(max(lo, min(hi, v))) 