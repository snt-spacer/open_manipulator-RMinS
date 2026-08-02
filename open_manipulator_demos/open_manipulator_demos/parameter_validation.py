from math import isfinite, sqrt
from typing import Iterable


def finite_float(name: str, value: float) -> float:
    number = float(value)
    if not isfinite(number):
        raise ValueError(f'{name} must be finite')
    return number


def positive_float(name: str, value: float) -> float:
    number = finite_float(name, value)
    if number <= 0.0:
        raise ValueError(f'{name} must be positive')
    return number


def scaling_factor(name: str, value: float) -> float:
    number = positive_float(name, value)
    if number > 1.0:
        raise ValueError(f'{name} must be in the range (0, 1]')
    return number


def finite_sequence(
    name: str,
    values: Iterable[float],
    expected_length: int,
) -> list[float]:
    numbers = [finite_float(name, value) for value in values]
    if len(numbers) != expected_length:
        raise ValueError(
            f'{name} must contain {expected_length} values, got {len(numbers)}'
        )
    return numbers


def positive_sequence(
    name: str,
    values: Iterable[float],
    expected_length: int,
) -> list[float]:
    numbers = finite_sequence(name, values, expected_length)
    if any(number <= 0.0 for number in numbers):
        raise ValueError(f'{name} values must be positive')
    return numbers


def normalized_quaternion(
    name: str,
    values: Iterable[float],
    tolerance: float = 1e-3,
) -> list[float]:
    quaternion = finite_sequence(name, values, 4)
    norm = sqrt(sum(component * component for component in quaternion))
    if abs(norm - 1.0) > tolerance:
        raise ValueError(f'{name} must be normalized, got norm {norm:.6f}')
    return quaternion


def non_empty_string(name: str, value: str) -> str:
    text = str(value).strip()
    if not text:
        raise ValueError(f'{name} cannot be empty')
    return text
