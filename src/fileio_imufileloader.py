from src.common_types import IMU
import numpy as np

# imufile需要isOpen endtime starttime next isEof close方法

class ImuFileLoader:
    def __init__(self, filepath, datalen, datarate):
        self.path = filepath
        self.dt_ = 1.0 / datarate
        self.imu_ = IMU()
        self.imu_pre_ = IMU()
        self.data_:np.array
        self.interation:int = 0 # 模拟后面的next迭代

    def open(self):
        with open(self.path, 'r', encoding='utf-8') as imufile:
            imudata = np.loadtxt(imufile)
        return imudata
    
    def load(self):
        imudata = self.open()
        self.data_ = imudata[self.interation]

    def next(self):
        self.load()

        self.imu_pre_ = self.imu_
        self.imu_.time = self.data_[0]
        self.imu_.dtheta = self.data_[1:4]
        self.imu_.dvel = self.data_[4:7]
        dt = self.imu_.time - self.imu_pre_.time
        self.imu_.dt = dt if dt < 0.1 else self.dt_
        self.interation = self.interation + 1
        return self.imu_
    
    def starttime(self):
        imudata = self.open()
        imudata = imudata[0]
        return imudata[0]
    
    def endtime(self):
        imudata = self.open()
        endcount = imudata.shape[0]
        return (imudata[endcount-1])[0]
        

    