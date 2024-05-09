import numpy as np
from enum import Enum
from src.kf_gins_kf_gins_type import GINSOptions
from src.common_types import IMU, GNSS
from src.kf_gins_kf_gins_type import PVA, ImuError

class GIEngine:
    def __init__(self, options):
        # 相关变量初始化
        self.options_ = GINSOptions()
        self.timestamp_ : np.double
        # 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
        # updata time align error
        self.TIME_ALIGN_ERR = 0.001
        # IMU和GNSS原始数据
        # raw imudata and gnssdata
        self.imupre_ = IMU()
        self.imucur_ = IMU()
        self.gnssdata_ = GNSS()
        # IMU状态（位置、速度、姿态和IMU误差）
        # imu state (position, velocity, attitude and imu error)
        self.pvacur_ = PVA()
        self.pvapre_ = PVA()
        self.imuerror_ = ImuError()
        # Kalman滤波相关
        # ekf variables
        self.Cov_ : np.array
        self.Qc_ : np.array
        self.dx_ : np.array

        self.RANK : int = 21
        self.NOISERANK : int = 18
        # 状态ID和噪声ID
        # state ID and noise ID
        self.StateID = Enum('StateID', (('P_ID', 0), ('V_ID', 3), ('PHI_ID', 6), ('BG_ID', 9), ('BA_ID', 12), ('SG_ID', 15), ('SA_ID', 18)))
        self.NoiseID = Enum('NoiseID', (('VRW_ID', 0), ('ARW_ID', 3), ('BGSTD_ID', 6), ('BASTD_ID', 9), ('SGSTD_ID', 12), ('SASTD_ID', 15)))

        # 相关操作初始化
        self.options_ = options
        self.options_.print_options()
        self.timestamp_ = 0.0
        # 设置协方差矩阵，系统噪声阵和系统误差状态矩阵大小
        # resize covariance matrix, system noise matrix, and system error state matrix
        self.Cov_ = np.zeros((self.RANK, self.RANK), dtype=np.double)
        self.Qc_ = np.zeros((self.NOISERANK, self.NOISERANK), dtype=np.double)
        self.dx_ = np.zeros((self.RANK, 1), np.double)
        # 初始化系统噪声阵
        # initialize noise matrix
        imunoise = self.options_.imunoise
        sub_gyr_arw = self.Qc_[self.NoiseID.ARW_ID.value:self.NoiseID.ARW_ID.value+3, self.NoiseID.ARW_ID.value:self.NoiseID.ARW_ID.value+3]
        sub_gyr_arw += np.diag(np.multiply(imunoise.gyr_arw, imunoise.gyr_arw))
        sub_acc_vrw = self.Qc_[self.NoiseID.VRW_ID.value:self.NoiseID.VRW_ID.value+3, self.NoiseID.VRW_ID.value:self.NoiseID.VRW_ID.value+3]
        sub_acc_vrw += np.diag(np.multiply(imunoise.acc_vrw, imunoise.acc_vrw))
        sub_gyrbias_std = self.Qc_[self.NoiseID.BGSTD_ID.value:self.NoiseID.BGSTD_ID.value+3, self.NoiseID.BGSTD_ID.value, self.NoiseID.BGSTD_ID.value+3]
        sub_gyrbias_std += 2 / np.diag(np.multiply(imunoise.gyrbias_std, imunoise.gyrbias_std))
        sub_accbias_std = self.Qc_[self.NoiseID.BASTD_ID.value:self.NoiseID.BASTD_ID.value+3, self.NoiseID.BASTD_ID.value:self.NoiseID.BASTD_ID.value+3]
        sub_accbias_std += 2 / np.diag(np.multiply(imunoise.accbias_std, imunoise.accbias_std))
        sub_gyrscale_std = self.Qc_[self.NoiseID.SGSTD_ID.value:self.NoiseID.SGSTD_ID.value+3, self.NoiseID.SGSTD_ID.value:self.NoiseID.SGSTD_ID.value+3]
        sub_gyrscale_std += 2 / np.diag(np.multiply(imunoise.gyrscale_std, imunoise.gyrscale_std))
        sub_accscale_std = self.Qc_[self.NoiseID.SASTD_ID.value:self.NoiseID.SASTD_ID.value+3, self.NoiseID.SASTD_ID.value:self.NoiseID.SASTD_ID.value+3]
        sub_accscale_std += 2 / np.diag(np.multiply(imunoise.accscale_std, imunoise.accscale_std))

        # 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
        # set initial state (position, velocity, attitude and IMU error) and covariance
        self.initialize(self.options_.initstate, self.options_.initstate_std)


    def addImuData(self, imu, compensate):
        imupre_ = imucur_
        imucur_ = imu
        if(compensate):
            self.imuCompensate(imucur_)

    def addGnssData(self, gnss):
        gnssdata_ = gnss
        gnssdata_.isvalid = True


    def newImuProcess(self):
        return 0

    def imuInterpolate(self, imu1, imu2, timestamp, midimu):
        if(imu1.time > timestamp or imu2.time < timestamp):
            return 0
        lamda : np.double = (timestamp - imu1.time) / (imu2.time - imu1.time)
        
        midimu.time = timestamp
        midimu.dtheta = imu2.dtheta * lamda
        midimu.dvel = imu2.dvel * lamda
        midimu.dt = timestamp - imu1.time

        imu2.dtheta = imu2.dtheta - midimu.dtheta
        imu2.dvel = imu2.dvel - midimu.dvel
        imu2.dt = imu2.dt - midimu.dt

    def timestamp(self):
        return self.timestamp

    def getNavState(self):
        return 0
    
    def getCovariance(self):
        return self.Cov_
    
    def initialize(self, initstate, initstate_std):
        # 初始化位置、速度、姿态
        # initialize position, velocity and attitude
        self.pvacur_.pos = initstate.pos
        self.pvacur_.vel = initstate.vel
        self.pvacur_.att.euler = initstate.euler
        self.pvacur_.att.cbn = # 欧拉角转矩阵 euler2matrix(self.pvacur_.att.euler)!!!!!!
        self.pvacur_.att.qbn = # 欧拉角转四元数 euler2quaternion(self.pvacur_.att.euler)!!!!!!
        # 初始化IMU误差
        # initialize imu error
        self.imuerror_ = initstate.imuerror
        # 给上一时刻状态赋同样的初值
        # set the same value to the previous state
        self.pvapre_ = self.pvacur_;      

        # 初始化协方差
        # initialize covariance
        imuerror_std = initstate_std.imuerror
        sub_init_std_pos = self.Cov_[self.StateID.P_ID.value:self.StateID.P_ID.value+3, self.StateID.P_ID.value:self.StateID.P_ID.value+3]
        sub_init_std_pos += np.diag(np.multiply(initstate_std.pos, initstate_std.pos))
        sub_init_std_vel = self.Cov_[self.StateID.V_ID.value:self.StateID.V_ID.value+3, self.StateID.V_ID.value:self.StateID.V_ID.value+3]
        sub_init_std_vel += np.diag(np.multiply(initstate_std.vel, initstate_std.vel))
        sub_init_std_euler = self.Cov_[self.StateID.PHI_ID.value:self.StateID.PHI_ID.value+3, self.StateID.PHI_ID.value:self.StateID.PHI_ID.value+3]
        sub_init_std_euler += np.diag(np.multiply(initstate_std.euler, initstate_std.euler))
        sub_init_std_gyrbias = self.Cov_[self.StateID.BG_ID.value:self.StateID.BG_ID.value+3, self.StateID.BG_ID.value:self.StateID.BG_ID.value+3]
        sub_init_std_gyrbias += np.diag(np.multiply(initstate_std.gyrbias, initstate_std.gyrbias))
        sub_init_std_accbias = self.Cov_[self.StateID.BA_ID.value:self.StateID.BA_ID.value+3, self.StateID.BA_ID.value:self.StateID.BA_ID.value+3]
        sub_init_std_accbias += np.diag(np.multiply(initstate_std.accbias, initstate_std.accbias))
        sub_init_std_gyrscale = self.Cov_[self.StateID.SG_ID.value:self.StateID.SG_ID.value+3, self.StateID.SG_ID.value:self.StateID.SG_ID.value+3]
        sub_init_std_gyrscale += np.diag(np.multiply(initstate_std.gyrscale, initstate_std.gyrscale))
        sub_init_std_accscale = self.Cov_[self.StateID.SA_ID.value:self.StateID.SA_ID.value+3, self.StateID.SA_ID.value:self.StateID.SA_ID.value+3]
        sub_init_std_accscale += np.diag(np.multiply(initstate_std.accscale, initstate_std.accscale))

    
    def imuCompensate(self, imu):
        return 0
    
    def isToUpdate(self, imutime1, imutime2, updatetime):
        return 0
    
    def insPropagation(self, imupre, imucur):
        return 0
    
    def gnssUpdate(self, gnssdate):
        return 0
    
    def EKFPredict(self, Phi, Qd):
        return 0
    
    def EKFUpdate(self, dz, H, R):
        return 0
    
    def stateFeedback(self):
        return 0
    
    def checkCov(self):
        for i in range(self.RANK):
            if(self.Cov_[i, i]<0):
                print("Covariance is negative at ", self.timestamp_, "!")
