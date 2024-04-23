# KF-GINS-PythonVersion
# Author : Jing Wang (jingwang.ac@hotmail.com)

# An EKF-Based GNSS/INS Integrated Navigation System
# The raw software is from i2Nav Group, Wuhan University
#   Raw Author :  Liqiang Wang (wlq@whu.edu.cn) 
# To satisfy the requirement under the Deep Learning environment 
# with Pytorch/Keras Lib, I transplanted the KF-GINS via Python
# programming language.

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
import numpy as np
import yaml

from src.common_angle import D2R, R2D
from src.kf_gins_kf_gins_type import GINSOptions
from src.fileio_gnssfileloader import GnssFileLoader
from src.fileio_imufileloader import ImuFileLoader





def main():
    # 加载配置文件并读取配置参数到GINSOptions中
    # load config file and load configuration parameters to GINSOptions
    configPath = "./dataset/kf-gins.yaml"
    with open(configPath, encoding='utf-8') as configFile:
        config = yaml.load(configFile, Loader=yaml.FullLoader)
    options = GINSOptions()
    loadConfig(config, options)
    
    # 读取文件路径配置
    # load filepath configuration
    imupath = config["imupath"]
    gnsspath = config["gnsspath"]
    outputpath = config["outputpath"]

    # imu数据配置 数据处理区间
    # imudata configuration data processing interval
    imudatalen = int(config["imudatalen"])
    imudatarate = int(config["imudatarate"])
    starttime = float(config["starttime"])
    endtime = float(config["endtime"])
    # 开始时间是456300 GNSS数据和IMU数据里的起始各不相同 因而接下来需要一个数据对其过程

    # 加载GNSS文件和IMU文件 (此部分尚未定义)
    # load GNSS file and IMU file
    gnssfile = GnssFileLoader(gnsspath)
    imufile = ImuFileLoader(imupath, imudatalen, imudatarate)

    # 构造GIEngine (此部分尚未定义)
    giengine = GIEngine(options)    


    # 检查文件是否打开
    # if(gnssfile.isOpen() or imufile.isOpen() or navfile.isOpen() or imuerrfile.isOpen() or stdfile.isOpen()):
    #     print("Failed to open data file!")
    # 先搭个能跑起来的框架 细节先不管了

    # 检查处理时间
    if(endtime<0):
        endtime = imufile.endtime()
    if(endtime > 604800 or starttime < imufile.satarttime() or starttime > endtime):
        print("Process Time Error !!")

    # 数据对齐
    # Data Alignment

    

    options.print_options()

def loadConfig(config, options):    ##options的类需要定义
    #  读取初始位置(纬度 经度 高程)、(北向速度 东向速度 垂向速度)、姿态(欧拉角，ZYX旋转顺序, 横滚角、俯仰角、航向角)
    #  load initial position(latitude longitude altitude)
    #               velocity(speeds in the directions of north, east and down)
    #               attitude(euler angle, ZYX, roll, pitch and yaw)
    
    # vec1~vec6是暂存变量的
    vec1 = (np.array(config["initpos"])).astype(np.double) 
    vec2 = (np.array(config["initvel"])).astype(np.double)
    vec3 = (np.array(config["initatt"])).astype(np.double)
    
    options.initstate.pos   = np.array([vec1[0], vec1[1], vec1[2]]) * D2R
    options.initstate.vel   = np.array([vec2[0], vec2[1], vec2[2]])
    options.initstate.euler = np.array([vec3[0], vec3[1], vec3[2]]) * D2R
    options.initstate.pos[2] *= R2D

    # 读取IMU误差初始值(零偏和比例因子)
    # load initial imu error (bias and scale factor)
    vec1 = (np.array(config["initgyrbias"])).astype(np.double)
    vec2 = (np.array(config["initaccbias"])).astype(np.double)
    vec3 = (np.array(config["initgyrscale"])).astype(np.double)
    vec4 = (np.array(config["initaccscale"])).astype(np.double)
    
    options.initstate.imuerror.gyrbias = np.array([vec1[0], vec1[1], vec1[2]])  * D2R / 3600.0
    options.initstate.imuerror.accbias = np.array([vec2[0], vec2[1], vec2[2]]) * 1e-5
    options.initstate.imuerror.gyrscale = np.array([vec3[0], vec3[1], vec3[2]]) * 1e-6
    options.initstate.imuerror.accscale = np.array([vec4[0], vec4[1], vec4[2]]) * 1e-6

    # load initial position std, velocity std and attitude(euler angle) std
    # 读取初始位置、速度和姿态(欧拉角)的标准差
    vec1 = (np.array(config["initposstd"])).astype(np.double)
    vec2 = (np.array(config["initvelstd"])).astype(np.double)
    vec3 = (np.array(config["initattstd"])).astype(np.double)
    
    options.initstate_std.pos = np.array([vec1[0], vec1[1], vec1[2]])
    options.initstate_std.vel = np.array([vec2[0], vec2[1], vec2[2]])
    options.initstate_std.euler = np.array([vec3[0], vec3[1], vec3[2]]) * D2R
    print("---Test---")
    print(options.initstate_std.pos)
    print("---Test---")
    # load IMU noise parameters
    # 读取IMU噪声参数
    vec1 = (np.array(config["imunoise"]["arw"])).astype(np.double)
    vec2 = (np.array(config["imunoise"]["vrw"])).astype(np.double)
    vec3 = (np.array(config["imunoise"]["gbstd"])).astype(np.double)
    vec4 = (np.array(config["imunoise"]["abstd"])).astype(np.double)
    vec5 = (np.array(config["imunoise"]["gsstd"])).astype(np.double)
    vec6 = (np.array(config["imunoise"]["asstd"])).astype(np.double)
    
    options.imunoise.corr_time = (np.array(config["imunoise"]["corrtime"])).astype(np.double)
    options.imunoise.gyr_arw = np.array([vec1[0], vec1[1], vec1[2]])
    options.imunoise.acc_vrw = np.array([vec2[0], vec2[1], vec2[2]])
    options.imunoise.gyrbias_std = np.array([vec3[0], vec3[1], vec3[2]])
    options.imunoise.accbias_std = np.array([vec4[0], vec4[1], vec4[2]])
    options.imunoise.gyrscale_std = np.array([vec5[0], vec5[1], vec5[2]])
    options.imunoise.accscale_std = np.array([vec6[0], vec6[1], vec6[2]])
    
    # load initial IMU bias and scale std, set to bias and scale instability std if load failed
    # 读取IMU误差初始标准差，如果配置文件中没有设置，则采用IMU噪声参数中的零偏和比例因子的标准差
    if("initbgstd" in config):
        vec1 = (np.array(config["initbgstd"])).astype(np.double)
    else:
        vec1 = options.imunoise.gyrbias_std
    if("initbastd" in config):
        vec2 = (np.array(config["initbastd"])).astype(np.double)
    else:
        vec2 = options.imunoise.accbias_std
    if("initsgstd" in config):
        vec3 = (np.array(config["initsgstd"])).astype(np.double)
    else:
        vec3 = options.imunoise.gyrscale_std
    if("initsastd" in config):
        vec4 = (np.array(config["initsastd"])).astype(np.double)
    else:
        vec4 = options.imunoise.accscale_std

    # convert initial IMU errors' units to standard units
    # IMU初始误差转换为标准单位
    options.initstate_std.imuerror.gyrbias = np.array([vec1[0], vec1[1], vec1[2]]) * D2R / 3600.0
    options.initstate_std.imuerror.accbias = np.array([vec2[0], vec2[1], vec2[2]]) * 1e-5
    options.initstate_std.imuerror.gyrscale = np.array([vec3[0], vec3[1], vec3[2]]) * 1e-6
    options.initstate_std.imuerror.accscale = np.array([vec4[0], vec4[1], vec4[2]]) * 1e-6
    
    # convert IMU noise parameters' units to standard units
    # IMU噪声参数转换为标准单位
    options.imunoise.gyr_arw *= (D2R / 60.0)
    options.imunoise.acc_vrw /= 60.0
    options.imunoise.gyrbias_std *= (D2R / 3600.0)
    options.imunoise.accbias_std *= 1e-5
    options.imunoise.gyrscale_std *= 1e-6
    options.imunoise.accscale_std *= 1e-6
    options.imunoise.corr_time *= 3600

    # gnss antenna leverarm, posotion of GNSS antenna phase center in IMU frame
    # GNSS天线杆臂， GNSS天线相位中心在IMU坐标系下位置
    if("antlever" in config):
        vec1 = (np.array(config["antlever"])).astype(np.double)
        options.antlever = vec1
    
    return True


main()

