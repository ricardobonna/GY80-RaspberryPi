#!/usr/bin/python

import time
import smbus           #import smbus to access i2c port
from math import pi


#Digital compass
class HMC5883L(object):

    REG_CONFIG_A     = 0x00
    REG_CONFIG_B     = 0x01
    REG_MODE         = 0x02
    REG_DATAX0       = 0x04
    REG_DATAX1       = 0x03
    REG_DATAY0       = 0x06
    REG_DATAY1       = 0x05
    REG_DATAZ0       = 0x08
    REG_DATAZ1       = 0x07
    
    def __init__(self, bus = 1, addr = 0x1E):
        self.i2c_bus = smbus.SMBus(bus)
        self.addr = addr
        self.i2c_bus.write_byte_data(self.addr, self.REG_MODE, 0x00)
        self.i2c_bus.write_byte_data(self.addr, self.REG_CONFIG_A, 0x74)

    #converts 16 bit two's compliment reading to signed int
    def getSignedNumber(self,number):
        if number & (1 << 15):
            return number | ~65535
        else:
            return number & 65535
    
    #read the HMC5883L data from the i2c port
    def get_Data(self):
        X_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAX0)
        X_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAX1)
        X = X_H << 8 | X_L
        
        Y_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAY0)
        Y_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAY1)
        Y = Y_H << 8 | Y_L
        
        Z_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAZ0)
        Z_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAZ1)
        Z = Z_H << 8 | Z_L
        
        X = self.getSignedNumber(X)
        Y = self.getSignedNumber(Y)
        Z = self.getSignedNumber(Z)
        
        return [X,Y,Z]


#Accelerometer sensor
class ADXL345(object):
    
    REG_DEVID        = 0x00 # Device ID
    REG_POWER_CTL    = 0x2D # Power-saving features control
    REG_DATA_FORMAT  = 0x31
    REG_DATAX0       = 0x32
    REG_DATAX1       = 0x33
    REG_DATAY0       = 0x34
    REG_DATAY1       = 0x35
    REG_DATAZ0       = 0x36
    REG_DATAZ1       = 0x37

    DATARATE_0_10_HZ = 0x00
    DATARATE_0_20_HZ = 0x01
    DATARATE_0_39_HZ = 0x02
    DATARATE_0_78_HZ = 0x03
    DATARATE_1_56_HZ = 0x04
    DATARATE_3_13_HZ = 0x05
    DATARATE_6_25HZ  = 0x06
    DATARATE_12_5_HZ = 0x07
    DATARATE_25_HZ   = 0x08
    DATARATE_50_HZ   = 0x09
    DATARATE_100_HZ  = 0x0A # (default)
    DATARATE_200_HZ  = 0x0B
    DATARATE_400_HZ  = 0x0C
    DATARATE_800_HZ  = 0x0D
    DATARATE_1600_HZ = 0x0E
    DATARATE_3200_HZ = 0x0F

    RANGE_2_G        = 0x00 # +/-  2g (default)
    RANGE_4_G        = 0x01 # +/-  4g
    RANGE_8_G        = 0x02 # +/-  8g
    RANGE_16_G       = 0x03 # +/- 16g
    
    def __init__(self, bus = 1, addr = 0x53):
        self.i2c_bus = smbus.SMBus(bus)
        self.addr = addr
        self.i2c_bus.write_byte_data(self.addr, self.REG_DATA_FORMAT, self.RANGE_2_G)
        # Enable the accelerometer
        self.i2c_bus.write_byte_data(self.addr, self.REG_POWER_CTL, 0x08)

    #converts 16 bit two's compliment reading to signed int
    def getSignedNumber(self,number):
        if number & (1 << 15):
            return number | ~65535
        else:
            return number & 65535
    
    #read the ADXL345 data from the i2c port
    def get_Data(self):
        X_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAX0)
        X_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAX1)
        X = X_H << 8 | X_L
        
        Y_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAY0)
        Y_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAY1)
        Y = Y_H << 8 | Y_L
        
        Z_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAZ0)
        Z_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAZ1)
        Z = Z_H << 8 | Z_L
        
        X = self.getSignedNumber(X)
        Y = self.getSignedNumber(Y)
        Z = self.getSignedNumber(Z)
        
        return [X,Y,Z]


#Gyro sensor
class L3G4200D(object):

    REG_CTRL_1       = 0x20
    REG_CTRL_2       = 0x21
    REG_CTRL_3       = 0x22
    REG_CTRL_4       = 0x23
    REG_CTRL_5       = 0x24
    REG_DATAX0       = 0x28
    REG_DATAX1       = 0x29
    REG_DATAY0       = 0x2A
    REG_DATAY1       = 0x2B
    REG_DATAZ0       = 0x2C
    REG_DATAZ1       = 0x2D
    
    def __init__(self, bus = 1, addr = 0x69):
        self.i2c_bus = smbus.SMBus(bus)
        self.addr = addr
        #initialize the L3G4200D
        #normal mode and all axes on to control reg1
        self.i2c_bus.write_byte_data(self.addr,self.REG_CTRL_1,0x0F)
        #2000dps to control reg4
        self.i2c_bus.write_byte_data(self.addr,self.REG_CTRL_4,0x20)

    #converts 16 bit two's compliment reading to signed int
    def getSignedNumber(self,number):
        if number & (1 << 15):
            return number | ~65535
        else:
            return number & 65535
    
    #read the L3G4200D data from the i2c port
    def get_Data(self):
        X_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAX0)
        X_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAX1)
        X = X_H << 8 | X_L
        
        Y_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAY0)
        Y_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAY1)
        Y = Y_H << 8 | Y_L
        
        Z_L = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAZ0)
        Z_H = self.i2c_bus.read_byte_data(self.addr,self.REG_DATAZ1)
        Z = Z_H << 8 | Z_L
        
        X = self.getSignedNumber(X)
        Y = self.getSignedNumber(Y)
        Z = self.getSignedNumber(Z)
        
        return [X,Y,Z]


################################################################
############### Integracao entre os dispositivos ###############
################################################################


class GY80(object):
    def __init__(self,bus = 1):
        self.gyro = L3G4200D(bus)
        self.accel = ADXL345(bus)
        self.mag = HMC5883L(bus)
        self.g_bias = 0
        self.calibrate_gyro()

    def calibrate_gyro(self):
        [X1,Y1,Z1] = self.gyro.get_Data()
        [X2,Y2,Z2] = self.gyro.get_Data()
        [X3,Y3,Z3] = self.gyro.get_Data()
        [X4,Y4,Z4] = self.gyro.get_Data()
        [X5,Y5,Z5] = self.gyro.get_Data()
        X = (X1+X2+X3+X4+X5)/5
        Y = (Y1+Y2+Y3+Y4+Y5)/5
        Z = (Z1+Z2+Z3+Z4+Z5)/5
        self.g_bias = [X,Y,Z]
        
    def read_gyro(self):        # rad/s
        [X,Y,Z] = self.gyro.get_Data()
        X = X - self.g_bias[0]
        Y = Y - self.g_bias[1]
        Z = Z - self.g_bias[2]
        X = round(X*2000*pi/(2**15*180),2)  # parece que esta funcionando
        Y = round(Y*2000*pi/(2**15*180),2)  # com 2300, mas deveria ser com
        Z = round(Z*2000*pi/(2**15*180),2)  # 2000 e nao sei pq   
        return [X,Y,Z]

    def read_accel(self):       # m/s^2
        [X,Y,Z] = self.accel.get_Data()
        X = round(X*9.81/2**8,2)
        Y = round(Y*9.81/2**8,2)
        Z = round(Z*9.81/2**8,2)
        return [X,Y,Z]

    def read_mag(self):         # mili Gauss
        [X,Y,Z] = self.mag.get_Data()
        X = round(X*0.92,2)
        Y = round(Y*0.92,2)
        Z = round(Z*0.92,2)
        return [X,Y,Z]
    
    def read_Devices(self):
        [Xg,Yg,Zg] = self.read_gyro()
        [Xa,Ya,Za] = self.read_accel()
        [Xm,Ym,Zm] = self.read_mag()
        return [[Xg,Yg,Zg],[Xa,Ya,Za],[Xm,Ym,Zm]]


if __name__ == "__main__":
    #testing the code
    sensor =  GY80()
    while True:
        [g,a,m] = sensor.read_Devices()
        print([g,a,m])
        time.sleep(0.1)
