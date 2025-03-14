class KalmanFilter:
    def __init__(self, process_variance: float, measurement_variance: float, initial_error_covariance: float, initial_estimate: float):
        """
        Initialize the Kalman filter.

        :param process_variance: The process (model) variance (q)
        :param measurement_variance: The measurement variance (r)
        :param initial_error_covariance: The initial error covariance (p)
        :param initial_estimate: The initial estimate (xHat)
        """
        self.q = process_variance
        self.r = measurement_variance
        self.p = initial_error_covariance
        self.f = 1.0  # State transition coefficient
        self.h = 1.0  # Observation model coefficient
        self.xHat = float(initial_estimate)

    def compute(self, measurement: float) -> float:
        """
        Perform a single Kalman filter update with the given measurement.

        :param measurement: The new measurement value.
        :return: The updated state estimate.
        """
        # Prediction step
        xHat_predict = self.f * self.xHat
        p_predict = self.f * self.p * self.f + self.q

        # Kalman gain calculation
        k = p_predict * self.h / (self.h * p_predict * self.h + self.r)

        # Update step
        self.xHat = xHat_predict + k * (float(measurement) - self.h * xHat_predict)
        self.p = (1 - k * self.h) * p_predict

        return self.xHat


class MultiKalmanFilter:
    def __init__(self, process_variance: float, measurement_variance: float, initial_error_covariance: float, initial_estimates: list[float]):
        """
        Initialize a collection of Kalman filters.

        :param process_variance: The process (model) variance (q) for each filter.
        :param measurement_variance: The measurement variance (r) for each filter.
        :param initial_error_covariance: The initial error covariance (p) for each filter.
        :param initial_estimates: A list of initial estimates, one per filter.
        """
        self.filters = [
            KalmanFilter(process_variance, measurement_variance, initial_error_covariance, est)
            for est in initial_estimates
        ]

    def compute(self, measurements: list[float]) -> list[float]:
        """
        Update each Kalman filter with the corresponding measurement.

        :param measurements: A list of measurement values, one per filter.
        :return: A list of updated state estimates.
        """
        results = []
        for kf, measurement in zip(self.filters, measurements):
            results.append(kf.compute(measurement))

        return results
