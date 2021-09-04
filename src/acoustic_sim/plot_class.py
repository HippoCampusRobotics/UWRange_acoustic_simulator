import numpy as np
import matplotlib.pyplot as plt

class plot():
    def __init__(self):
        self.GPSx = []
        self.GPSy = []
        self.Filterx = []
        self.Filtery = []
        self.timeGPS = []
        self.timeFilter = []
        self.sim_Meas1 = []
        self.time1 = []
        self.sim_Meas2 = []
        self.time2 = []
        self.sim_Meas3 = []
        self.time3 = []
        self.sim_Meas4 = []
        self.time4 = []
        self.ID = []
        self.Anchor1 = [42.168999999994412 , 26.654000000096858]
        self.Anchor2 = [18.150999999954365 , 64.179000000469387]
        self.Anchor3 = [-24.253000000026077 ,37.628999999724329]
        self.Anchor4 = [0, 0]
        

    def plotDistError(self, measDist, x):
        pass

    def addMeas(self, dist, time, ID):
        if ID == 1:
            self.sim_Meas1.append(dist)
            self.time1.append(time)
        elif ID == 2:
            self.sim_Meas2.append(dist)
            self.time2.append(time)
        elif ID == 3:
            self.sim_Meas3.append(dist)
            self.time3.append(time)
        elif ID == 4:
            self.sim_Meas4.append(dist)
            self.time4.append(time)
            

    def addPosFilter(self, time, XFilter):
        self.Filterx.append(XFilter[0])
        self.Filtery.append(XFilter[1])
        self.timeFilter.append(time)

    def addPosGPS(self, time, x):
        self.GPSx.append(x[0])
        self.GPSy.append(x[1])
        self.timeGPS.append(time)

    def plotPath(self):
        plt.figure(0)
        plt.scatter(self.GPSx, self.GPSy, s = 2, color = "k")
        plt.scatter(self.Filterx,self.Filtery, s = 2, c = self.timeGPS)
        plt.scatter(self.Anchor1[0], self.Anchor1[1], marker="X", color ="y")
        plt.scatter(self.Anchor2[0], self.Anchor2[1], marker="X", color ="b")
        plt.scatter(self.Anchor3[0], self.Anchor3[1], marker="X", color ="g")
        plt.scatter(self.Anchor4[0], self.Anchor4[1], marker="X", color ="r")
        plt.annotate("A1", self.Anchor1, color = "y", )
        plt.annotate("A2", self.Anchor2, color = "b", )
        plt.annotate("A3", self.Anchor3, color = "g")
        plt.annotate("A4", self.Anchor4, color = "r")
        plt.title("Path GPS/ Filter")
        plt.xlabel("X-Koordinaten [m]")
        plt.ylabel("Y-Koordinaten [m]")
        plt.colorbar(label = "Time [s]")
    
    def plotXoverTime(self):
        plt.figure(3)
        plt.scatter(self.timeGPS, self.Filterx)
        plt.xlabel("Time [s]")
        plt.ylabel("X-Koord [m]")

    def plotErroverTime(self):
        plt.figure(4)
        time = []
        errX = []
        errY = []
        for i in range(len(self.timeGPS)-1):
            time.append(self.timeGPS[i])
            errX.append(self.GPSx[i]-self.Filterx[i])
            errY.append(self.GPSy[i]-self.Filtery[i])
        plt.plot(time, errX, "rx-", label = "Error in X Direction")
        plt.plot(time, errY, "bx-", label = "Error in Y Direction")
        plt.xlabel("Time [s]")
        plt.ylabel("Absolut Error between GPS Path and Estimated Path")


    def plotMeas(self, meas_data):
        meas = np.array(meas_data).T
        time_meas1 = []
        dist1 = []
        time_meas2 = []
        dist2 = []
        time_meas3 = []
        dist3 = []
        time_meas4 = []
        dist4 = []
        
        for i in range(len(meas)):
            if meas_data[2][i] == 1:
                time_meas1.append(meas_data[0][i])
                dist1.append(float(meas_data[1][i]))

            elif meas_data[2][i] == 2:
                time_meas2.append(meas_data[0][i])
                dist2.append(float(meas_data[1][i]))

            elif meas_data[2][i] == 3:
                time_meas3.append(meas_data[0][i])
                dist3.append(float(meas_data[1][i]))
            
            elif meas_data[2][i] == 4:
                time_meas4.append(meas_data[0][i])
                dist4.append(float(meas_data[1][i]))


        plt.figure(1)
        plt.subplot(4,1,1)
        plt.scatter(self.time1, self.sim_Meas1, color = "y", label ="Anchor 1 simulated")
        plt.scatter(time_meas1, dist1, color="k", label ="Anchor 1 measured")
        plt.legend(loc="upper right")
        plt.xlabel("time [s]")
        plt.ylabel("Measured distance [m]")

        plt.subplot(4,1,2)
        plt.scatter(self.time2, self.sim_Meas2, color = "b", label = "Anchor 2 simulated")
        plt.scatter(time_meas2, dist2, color="k", label ="Anchor 2 measured")
        plt.legend(loc="upper right")
        plt.xlabel("time [s]")
        plt.ylabel("Measured distance [m]")

        plt.subplot(4,1,3)
        plt.scatter(self.time3, self.sim_Meas3, color = "g", label="Anchor 3 simulated")
        plt.scatter(time_meas3, dist3, color="k", label = "Anchor 3 measured")
        plt.legend(loc="upper right")
        plt.xlabel("time [s]")
        plt.ylabel("Measured distance [m]")

        plt.subplot(4,1,4)
        plt.scatter(self.time4, self.sim_Meas4, color = "r", label="Anchor 4 simulated")
        plt.scatter(time_meas4, dist4, color="k", label = "Anchor 4 measured")
        plt.legend(loc="upper right")
        plt.xlabel("time [s]")
        plt.ylabel("Measured distance [m]")


    def plot(self, meas_data):
        self.plotPath()
        self.plotMeas(meas_data)
        #self.plotXoverTime()
        self.plotErroverTime()
