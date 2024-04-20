class GNSS:
    def __init__(self):
        time: float
        blh: np.array
        std: np.array
        isvalid: bool

class IMU:
    def __init__(self):
        time: float
        dt: float
        dtheta: np.array
        dvel: np.array
        odovel: float

class Pose:
    def __init__(self):
        R: np.array
        t: np.array