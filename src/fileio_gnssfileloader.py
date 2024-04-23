from src.common_angle import D2R
from src.common_types import GNSS
import numpy as np

# 根据原kf_gins里的功能，首先得有isOpen方法，然后要调用next方法迭代，isEof判断是否到末尾，close关闭方法

class GnssFileLoader:
    def __init__(self, filepath, columns=7):
        self.data_: np.array
        self.path = filepath
        self.gnss_ = GNSS()
        # self.gnssfile = self.open(filepath, columns)
        self.interation:int = 0 # 模拟后面的next迭代

    def open(self):
        with open(self.path, 'r', encoding='utf-8') as gnssfile:
            gnssdata = np.loadtxt(gnssfile)
        return gnssdata
        # 只打开文件就好
        
    # def isOpen(self):
    #     return bool(self.gnssfile)
    
    def load(self):
        gnssdata = self.open()
        self.data_ = gnssdata[self.interation]
        # 加载一行文件内容到self.data_ 用interaion计数器来决定取哪一行
        
    def next(self):
        self.load()

        self.gnss_.time = self.data_[0]
        self.gnss_.blh = self.data_[1:4]

        # 13列GNSS文件包含GNSS速度
        if (self.data_.shape[0] == 7):
            self.gnss_.std = self.data_[4:7]
        else:
            self.gnss_.std = self.data_[7:10]

        # 将纬度和经度从度转换为弧度
        self.gnss_.blh[0] *= D2R
        self.gnss_.blh[1] *= D2R

        self.interation = self.interation + 1

        return self.gnss_


