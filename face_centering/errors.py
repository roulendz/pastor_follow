class FaceDetectionError(Exception):
    """Raised when face detection fails or yields invalid data."""


class CoordinateError(Exception):
    """Raised when coordinate mapping/calculation encounters invalid inputs."""


class MovementError(Exception):
    """Raised when movement execution fails at the controller/Arduino layer."""