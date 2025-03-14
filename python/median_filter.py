class MedianFilter:
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.values = []  # holds the sliding window values

    def compute(self, value: float) -> float:
        # Append the new value
        self.values.append(value)

        # If we exceed the window size, remove the oldest value
        if len(self.values) > self.window_size:
            self.values.pop(0)

        # Create a sorted copy of the values for median calculation
        sorted_values = sorted(self.values)
        n = len(sorted_values)

        # Compute the median: if odd, take the middle; if even, average the two middle elements.
        if n % 2 == 1:
            return sorted_values[n // 2]
        else:
            return (sorted_values[n // 2 - 1] + sorted_values[n // 2]) / 2.0


class MultiMedianFilter:
    def __init__(self, window_size: int, num_signals: int):
        # Create a list of MedianFilter instances, one for each signal.
        self.filters = [MedianFilter(window_size) for _ in range(num_signals)]

    def compute(self, values: list[float]) -> list[float]:
        # For each signal (and its corresponding filter), compute the filtered median.
        results = []
        for filt, value in zip(self.filters, values):
            results.append(filt.compute(value))
        return results
