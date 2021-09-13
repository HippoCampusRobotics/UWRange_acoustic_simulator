#!/usr/bin/env python

import csv
import numpy as np
import os 
from pyproj import Proj
from tqdm import tqdm

tmp = os.path.dirname(__file__)

# Input time = UTC, Position = UTM
class dataLoader():
    def __init__(self):
    
        """ Config file inclued all scalable params of UKF, Acoustic Simulation, 
        and global frame params"""
        # #def off ZeroPoint
        # zeroLat = self.TotalToDeci(535366836)
        # zeroLon = self.TotalToDeci(98706027)
        # self.UTMLon0, self.UTMLat0 = self.calculateUTM(zeroLon, zeroLat)
        
        # # Anchor 3
        # a3Lat = self.TotalToDeci(535365282)
        # a3Lon = self.TotalToDeci(98707567)
        # self.UTMLonA3, self.UTMLatA3 = self.calculateUTM(a3Lon, a3Lat)

        # # Anchor 4
        # a4Lat = self.TotalToDeci(535357250)
        # a4Lon = self.TotalToDeci(98718780)
        # self.UTMLonA4, self.UTMLatA4 = self.calculateUTM(a4Lon, a4Lat)
    
    
        # print(f"Anchor2: {self.UTMLon0-self.UTMLon0}/{self.UTMLat0-self.UTMLat0}")
        # print(f"Anchor3: {self.UTMLonA3-self.UTMLon0}/{self.UTMLatA3-self.UTMLat0}")
        # print(f"Anchor4: {self.UTMLonA4-self.UTMLon0}/{self.UTMLatA4-self.UTMLat0}")
        

    def writeNewFile(self):
       
        mat_gps_newfile = []
        with open(os.path.join(tmp,"Data/circle/circle_Gps.csv")) as f:
        #with open(os.path.join(tmp,"Data/alternating/alternating_Gps.csv")) as f:
            reader = csv.reader(f, delimiter = ",")
            for row in reader:
                mat_gps_newfile.append(row)

        lengMat = len(mat_gps_newfile)
        time_gps = [0]*lengMat
        gpsPosx = [0]*lengMat
        utmPosx = [0]*lengMat
        gpsPosy = [0]*lengMat
        utmPosy = [0]*lengMat
        
        with open((os.path.join(tmp,"Data/circle/circle_Gps_utm.csv")), mode = "w", newline="") as f:
        #with open((os.path.join(tmp,"Data/alternating/alternating_Gps_utm.csv")), mode = "w", newline="") as f:
            writer = csv.writer(f, delimiter=",")
            for i in tqdm(range(lengMat), ncols=100):
                time_gps[i] = mat_gps_newfile[i][0]
                gpsPosy[i] = self.TotalToDeci(mat_gps_newfile[i][1])
                gpsPosx[i] = self.TotalToDeci(mat_gps_newfile[i][2])
                utmPosx[i], utmPosy[i] = self.calculateUTM(gpsPosx[i], gpsPosy[i])
                writer.writerow([time_gps[i], utmPosy[i], utmPosx[i]])


    def inputGpsUtc(self):
        mat_gps = []
        #with open(os.path.join(tmp,"Data/circle/circle_Gps_utm.csv")) as f:
        with open(os.path.join(tmp,"Data/real_Gpstrack_100Hz.csv")) as f:
            reader = csv.reader(f, delimiter = ",")
            for row in reader:
                mat_gps.append(row)

        lengMat = len(mat_gps)
        time_gps = [0]*lengMat
        utmPosx = [0]*lengMat
        utmPosy = [0]*lengMat
        utmPosx[0] = float(mat_gps[0][2]) - self.UTMLon0
        utmPosy[0] = float(mat_gps[0][1]) - self.UTMLat0
        depth = [0]*lengMat
        t0 = self.calculateTimestamp(mat_gps[0][0])
        vx =[0]*lengMat
        vy =[0]*lengMat
        vz = [0]*lengMat

        for i in tqdm(range(lengMat-1), ncols=100):
            time_gps[i] = np.round(self.calculateTimestamp(mat_gps[i+1][0])- t0,6)
            dt = np.round(self.calculateTimestamp(mat_gps[i+1][0])- self.calculateTimestamp(mat_gps[i][0]),2)
            utmPosx[i+1] = float(mat_gps[i+1][2]) - self.UTMLon0
            utmPosy[i+1] = float(mat_gps[i+1][1]) - self.UTMLat0
            depth[i] = 0.5
            if dt == 0:
                vx[i], vy[i], vz[i] = 0,0,0
            else:
                vx[i] = np.round((utmPosx[i+1] - utmPosx[i])/dt,4)
                vy[i] = np.round((utmPosy[i+1] - utmPosy[i])/dt,4)
                vz[i] = 0
    
        mat_meas = []
        #with open(os.path.join(tmp,"Data/circle/circle_meas.csv")) as f:
        with open(os.path.join(tmp,"Data/meas.csv")) as f:
            reader = csv.reader(f, delimiter = ",")
            for row in reader:
                mat_meas.append(row)
       
        lengMat_meas = len(mat_meas)
        time_meas = [0]*lengMat_meas
        t0_meas = self.calculateTimestamp(mat_meas[0][0])
        dist = [0]*lengMat_meas
        AnchorID = [0]*lengMat_meas
        for i in range(lengMat_meas):
            time_meas[i] = np.round(self.calculateTimestamp(mat_meas[i][0])- t0_meas,2)
            dist[i] = mat_meas[i][1]
            AnchorID[i] = mat_meas[i][2]
        meas = [time_meas,dist,AnchorID]
        return time_gps, utmPosx, utmPosy, depth, vx, vy, vz, meas

    def inputClearedData(self):
        mat_gps = []
        with open(os.path.join(tmp,"Data/100Hz_circle20m_02513ms.csv")) as f:
            reader = csv.reader(f, delimiter = ",")
            for row in reader:
                mat_gps.append(row)

        lengMat = len(mat_gps)
        time_gps = [0]*lengMat
        Posx = [0]*lengMat
        Posy = [0]*lengMat
        depth = [0]*lengMat
        vx =[0]*lengMat
        vy =[0]*lengMat
        vz = [0]*lengMat
        Posx[0] = float(mat_gps[0][1])
        Posy[0] = float(mat_gps[0][2])
        depth[0] = float(mat_gps[0][3])

        for i in tqdm(range(lengMat-1), ncols=100):
            time_gps[i] = float(mat_gps[i+1][0])
            dt = float(mat_gps[i+1][0])- float(mat_gps[i][0])
            Posx[i+1] = float(mat_gps[i+1][1])
            Posy[i+1] = float(mat_gps[i+1][2])
            depth[i+1] = float(mat_gps[i+1][3])
            if dt == 0:
                vx[i], vy[i], vz[i] = 0,0,0
            else:
                vx[i] = (Posx[i+1] - Posx[i])/dt
                vy[i] = (Posy[i+1] - Posy[i])/dt
                vz[i] = (depth[i+1]- depth[i])/dt
    
        mat_meas = []
        
        # with open(os.path.join(tmp,"Data/meas.csv")) as f:
        #     reader = csv.reader(f, delimiter = ",")
        #     for row in reader:
        #         mat_meas.append(row)
       
        # lengMat_meas = len(mat_meas)
        # time_meas = [0]*lengMat_meas
        # dist = [0]*lengMat_meas
        # AnchorID = [0]*lengMat_meas
        # for i in tqdm(range(lengMat_meas), ncols=100):
        #     time_meas[i] = float(mat_meas[i][0])
        #     dist[i] = float(mat_meas[i][1])
        #     AnchorID[i] = int(mat_meas[i][2])
        meas = None#[time_meas,dist,AnchorID]
        return time_gps, Posx, Posy, depth, vx, vy, vz, meas
    
    def calculateTimestamp(self, UTCTime):
        if ":" in UTCTime:
            stdminsec = UTCTime.split(":")
            std = float(stdminsec[0])*60*60
            min = float(stdminsec[1])*60
            sec = float(stdminsec[2])
            timestamp = std + min + sec
        else:
            timestamp = float(UTCTime)
        return timestamp

    def calculateUTM(self, Lon, Lat):
        # Compute UTM Zone
        e2u_zone = int(divmod(Lon, 6)[0])+31
        # Converter defined
        e2u_conv = Proj(proj = 'utm', zone=e2u_zone, ellps = 'WGS84')
        # Comute Convertion
        utmx, utmy = e2u_conv(Lon, Lat)
        if Lat <0:
            utmy = utmy + 10000000
        
        return utmx, utmy
    
    def TotalToDeci(self, input):
        # number of decimal places
        input = str(input)
        dp = 7
        if "."  not in input:
                input = int(input)
                output = input/10**dp
        else:
            output = float(input)
        return output
               
def main():
    x = dataLoader()
    #x.writeNewFile()
    x.input()

if __name__=="__main__":
    main()