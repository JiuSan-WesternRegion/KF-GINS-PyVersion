import numpy as np
import quaternion

from src.common_angle import D2R
from src.common_angle import R2D


class Attitude:
    # qbn: np.quaternion
    # cbn: np.array
    # euler: np.array
    def __init__(self, qbn, cbn, euler):
        self.qbn = np.quaterniond      #np.quaterniond
        self.cbn = np.array      #np.array   matrix3d
        self.euler = np.array  #np.array   vector3d    默认列向量

class PVA:
    # pos: np.array
    # vel: np.array
    def __init__(self, pos, vel):
        self.pos = np.array      #np.array   vector3d
        self.vel = np.array      #np.array   vector3d

class ImuError:
    # gyrbias: np.array
    # accbias: np.array
    # gyrscale: np.array
    # accscale: np.array
    def __init__(self):
        self.gyrbias: np.array      #np.array   vector3d
        self.accbias: np.array      #np.array   vector3d
        self.gyrscale: np.array     #np.array   vector3d
        self.accscale: np.array     #np.array   vector3d

class NavState:
    # pos: np.array
    # vel: np.array
    # euler: np.array
    # imuerror = ImuError()
    def __init__(self):
        self.pos: np.array      #np.array   vector3d
        self.vel: np.array      #np.array   vector3d
        self.euler: np.array  #np.array   vector3d
        self.imuerror = ImuError()

class ImuNoise:
    # gyr_arw: np.array
    # acc_vrw: np.array
    # gyrbias_std: np.array
    # accbias_std: np.array
    # gyrscale_std: np.array
    # accscale_std: np.array
    # corr_time: np.double
    def __init__(self):
        self.gyr_arw: np.array              #np.array   vector3d
        self.acc_vrw: np.array              #np.array   vector3d
        self.gyrbias_std: np.array          #np.array   vector3d
        self.accbias_std: np.array          #np.array   vector3d
        self.gyrscale_std: np.array         #np.array   vector3d
        self.accscale_std: np.array         #np.array   vector3d
        self.corr_time: np.double           #double

class GINSOptions:
    # 注意 原先写法有问题 类变量重复引用造成变量错误 正确方法是改为实例变量
    # initstate = NavState()
    # initstate_std = NavState()
    
    # initstate = NavState()
    # initstate_std = NavState()
    # imunoise = ImuNoise()
    # antlever: np.array
    def __init__(self):
        self.initstate = NavState()
        self.initstate_std = NavState()
        self.imunoise = ImuNoise()
        self.antlever: np.array
    def print_options(self):
        print("---------------KF-GINS Options:---------------")
        # print initial state 打印初始状态
        print(" - Initial State: ")
        print("\t- initial position: ", self.initstate.pos[0]*R2D, self.initstate.pos[1]*R2D, self.initstate.pos[2], "[deg, deg, m]")
        print("\t- initial velocity: ", self.initstate.vel, "[m/s]")
        print("\t- initial attitude: ", self.initstate.euler*R2D, "[deg]")
        print("\t- initial gyrbias : ", self.initstate.imuerror.gyrbias*R2D*3600, "[deg/h]")
        print("\t- initial accbias : ", self.initstate.imuerror.accbias*1e5, "[mGal]")
        print("\t- initial gyrscale: ", self.initstate.imuerror.gyrscale*1e6, "[ppm]")
        print("\t- initial accscale: ", self.initstate.imuerror.accscale*1e6, "[ppm]")
        # print initial state STD 打印初始状态标准差
        print(" - Initial State STD: ")
        print("\t- initial position std: ", self.initstate_std.pos, "[deg, deg, m]")
        print("\t- initial velocity std: ", self.initstate_std.vel, "[m/s]")
        print("\t- initial attitude std: ", self.initstate_std.euler*R2D, "[deg]")
        print("\t- initial gyrbias std : ", self.initstate_std.imuerror.gyrbias*R2D*3600, "[deg/h]")
        print("\t- initial accbias std : ", self.initstate_std.imuerror.accbias*1e5, "[mGal]")
        print("\t- initial gyrscale std: ", self.initstate_std.imuerror.gyrscale*1e6, "[ppm]")
        print("\t- initial accscale std: ", self.initstate_std.imuerror.accscale*1e6, "[ppm]")
        # print IMU noise parameters 打印IMU噪声参数
        print(" - IMU noise: ")
        print("\t- arw:", self.imunoise.gyr_arw*R2D*60, "[deg/sqrt(h)]")
        print("\t- vrw:", self.imunoise.acc_vrw*60, "[m/s/sqrt(h)]")
        print("\t- gyrbias std :", self.imunoise.gyrbias_std*R2D*3600, "[deg/h]")
        print("\t- accbias std :", self.imunoise.accbias_std*1e5, "[mGal]")
        print("\t- gyrscale std:", self.imunoise.gyrscale_std*1e6, "[ppm]")
        print("\t- accscale std:", self.imunoise.accscale_std*1e6, "[ppm]")
        print("\t- correlation time:", self.imunoise.corr_time/3600.0, "[h]")

        # print GNSS antenna leverarm 打印GNSS天线杆臂
        print(" - Antenna leverarm: ", self.antlever, "[m]")

