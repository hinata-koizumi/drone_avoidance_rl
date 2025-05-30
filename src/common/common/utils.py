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
    return max(lo, min(hi, v)) 