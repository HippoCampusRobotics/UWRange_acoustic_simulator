#!/usr/bin/env python

from datetime import time
from tqdm import tqdm
import numpy as np
from acoustic_sim.datawriter_class import datawriter
import rospy
import threading
from acoustic_sim.acoustic_sim_class import acousticSimulation
from acoustic_sim.localisation_sim_class import localisationSimulation
from acoustic_sim.dataloader_class import dataLoader
from acoustic_sim.plot_class import plot
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from rospy.topics import Publisher
from std_msgs.msg import Float32
from sensor_msgs.msg import FluidPressure
from acoustic_sim.msg import ModemOut
import matplotlib.pyplot as plt
import json
import os

class simulation():
    def __init__(self):
        self.plot1 = plot()
        tmp = os.path.dirname(__file__)
        file_path_filter = os.path.join(tmp, '../config/acoustic_config.json')
        f = open(file_path_filter)
        self.acoustic_config = json.load(f)
        f.close()

        file_path_filter = os.path.join(tmp, '../config/filter_config.json')
        f = open(file_path_filter)
        self.filter_config = json.load(f)
        f.close()

        self.t0 = 0
        self.dt = 0
        self.t = self.t0
        self.last_t = self.t0
        self.z = None
        self.x0 = self.filter_config["config"][1]["settings"]["InitState"]
        self.x = None
        self.x_est = None
        self.statePre = None
        self.covarPre = None
        self.p_mat = None
        self.ros = self.acoustic_config["config"][0]["RosRun"]

        self.lock = threading.RLock()
        self.position = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.depth = 0

        # settings from Configfile
        self.f_pre = self.filter_config["config"][0]["PredictFrequency"]
        self.f_ac = self.acoustic_config["config"][0]["FrequencyAcousticSim"]
        
        self.acoustic_sim = acousticSimulation()
        self.localisation_sim = localisationSimulation()
        self.dataloader = dataLoader()
        self.dataWriter = datawriter()

        if self.ros:
            rospy.init_node("simulation")
            self.position_pub = rospy.Publisher("predict_state", PointStamped, queue_size=1)
            self.position_upd_pub0 = rospy.Publisher("update_state0", PointStamped, queue_size=1)
            self.position_upd_pub1 = rospy.Publisher("update_state1", PointStamped, queue_size=1)
            self.position_upd_pub2 = rospy.Publisher("update_state2", PointStamped, queue_size=1)
            self.position_upd_pub3 = rospy.Publisher("update_state3", PointStamped, queue_size=1)
            self.errAbs = rospy.Publisher("errAbs", PointStamped, queue_size=1)
            self.ModemOut0 = rospy.Publisher("ModemOut0", ModemOut, queue_size= 1)
            self.ModemOut1 = rospy.Publisher("ModemOut1", ModemOut, queue_size= 1)
            self.ModemOut2 = rospy.Publisher("ModemOut2", ModemOut, queue_size= 1)
            self.ModemOut3 = rospy.Publisher("ModemOut3", ModemOut, queue_size= 1)
            self.acousticError0 = rospy.Publisher("AcouERR0", ModemOut, queue_size=1)
            self.acousticError1 = rospy.Publisher("AcouERR1", ModemOut, queue_size=1)
            self.acousticError2 = rospy.Publisher("AcouERR2", ModemOut, queue_size=1)
            self.acousticError3 = rospy.Publisher("AcouERR3", ModemOut, queue_size=1)
            self.Anchor0 = rospy.Publisher("PosAnchor0", PointStamped, queue_size=1)
            self.Anchor1 = rospy.Publisher("PosAnchor1", PointStamped, queue_size=1)
            self.Anchor2 = rospy.Publisher("PosAnchor2", PointStamped, queue_size=1)
            self.Anchor3 = rospy.Publisher("PosAnchor3", PointStamped, queue_size=1)
            self.velocityPub = rospy.Publisher("TotalVelocity", PointStamped, queue_size=1)

            if self.acoustic_config["config"][0]["SimulationPath"]:
                self.position_sub = rospy.Subscriber("/bluero/ground_truth/state", Odometry, self.subscrib_position)
            else:
                self.position_sub = rospy.Subscriber("ground_truth/state", Odometry, self.subscrib_position)
            self.depth_sub = rospy.Subscriber("depth", Float32, self.subscrib_depth)
    
    def subscrib_position(self, msg: Odometry):
        pos = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:
            self.position = [pos.x, pos.y, pos.z]
            self.velocity = [v.x, v.y, v.z]
    
    def subscrib_depth(self, msg: Float32):
        with self.lock:
            self.depth = msg.data
 

    def publish_position(self, position, t, publisher: rospy.Publisher):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.from_sec(t)
        msg.header.frame_id = "map"
        msg.point.x = position[0]
        msg.point.y = position[1]
        msg.point.z = position[2]
        publisher.publish(msg)

    def publish_acousticMeas(self,index, meas, t, publisher: rospy.Publisher):
        msg = ModemOut()
        msg.dist = meas
        msg.id = index
        msg.timestamp = rospy.Time.from_sec(t)
        publisher.publish(msg)
    
    def publish_DistError(self, Error, t, AnchorID, publisher: rospy.Publisher):
        self.dataWriter.fillErr(Error)
        msg = ModemOut()
        msg.dist = Error
        msg.id = AnchorID
        msg.timestamp = rospy.Time.from_sec(t)
        publisher.publish(msg)
    
    def publish_PosError(self, x, t, x_est, publisher: rospy.Publisher):
        dx = x_est - x
        msg = PointStamped()
        msg.header.stamp = rospy.Time.from_sec(t)
        msg.header.frame_id = "map"
        msg.point.x = dx[0]
        msg.point.y = dx[1]
        msg.point.z = dx[2]
        publisher.publish(msg)
    
    def publish_totalVelo(self, t, publisher: rospy.Publisher):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.from_sec(t)
        msg.header.frame_id = "map"
        v = np.linalg.norm(self.velocity)
        msg.point.x = v
        msg.point.y = 0
        msg.point.z = 0
        publisher.publish(msg)
    
    def measErrDist(self, AnchorPos, meas, x):
        x = np.array(x)
        zhat = np.linalg.norm(AnchorPos-x)
        err = meas - zhat
        return err, zhat

    # def getAnchorPos(self, BeaconIndex):
    #     for i in self.acoustic_config["config"]:
    #         if i["type"] == "anchor":
    #             if i["modem"]["id"] == BeaconIndex:
    #                 return i["position"]

    def run(self):
        if self.ros:
            r = rospy.Rate(self.acoustic_config["config"][0]["FrequencyAcousticSim"])
            counter = 1
            steps = round(self.f_ac/ self.f_pre,0)
            while not rospy.is_shutdown():
                with self.lock:
                    x = self.position
                    preInput = self.velocity + np.random.normal(self.filter_config["config"][0]["MeasErrLoc"],self.filter_config["config"][0]["MeasErrScale"],3)
                    depth = self.depth
                    t = rospy.get_time()
                    meas = self.acoustic_sim.simulate(x, t)
                    self.dataWriter.fillState(x[0],x[1],x[2],t)
                    self.dataWriter.writeCSVState()
                    self.publish_totalVelo(t, self.velocityPub)
                    if meas is not None:
                        self.dataWriter.fillMeas(meas["time_published"], meas["tr"], meas["dist"], meas["realDist"], meas["ModemID"], meas["Error"])
                        xupd = self.localisation_sim.locate(preInput, t, depth, meas)
                        #print("Xupd: ",xupd)
                        #self.publish_position(xupd, t, self.position_pub)
                        # meas: 0-Index, 1-dist, 2-time
                        # {"dist": dist, "time_published": exittime, "ModemID": ID}
                        if meas["ModemID"] == 1:
                            self.publish_acousticMeas(meas["ModemID"],meas["dist"],meas["time_published"], self.ModemOut0)
                            self.publish_position(xupd, t, self.position_upd_pub0)
                            self.publish_position(xupd, t, self.position_pub)
                            self.publish_DistError(meas["Error"], t, meas["ModemID"], self.acousticError0)
                            self.publish_position(meas["ModemPos"], t, self.Anchor0)
                        elif meas["ModemID"] == 2:
                            self.publish_acousticMeas(meas["ModemID"],meas["dist"],meas["time_published"], self.ModemOut1)
                            self.publish_position(xupd, t, self.position_upd_pub1)
                            self.publish_position(xupd, t, self.position_pub)
                            self.publish_DistError(meas["Error"], t, meas["ModemID"], self.acousticError1)
                            self.publish_position(meas["ModemPos"], t, self.Anchor1)
                        elif meas["ModemID"] == 3:
                            self.publish_acousticMeas(meas["ModemID"],meas["dist"],meas["time_published"], self.ModemOut2)
                            self.publish_position(xupd, t, self.position_upd_pub2)
                            self.publish_position(xupd, t, self.position_pub)
                            self.publish_DistError(meas["Error"], t, meas["ModemID"], self.acousticError2)
                            self.publish_position(meas["ModemPos"], t, self.Anchor2)
                        elif meas["ModemID"] == 4:
                            self.publish_acousticMeas(meas["ModemID"],meas["dist"],meas["time_published"], self.ModemOut3)
                            self.publish_position(xupd, t, self.position_upd_pub3)
                            self.publish_position(xupd, t, self.position_pub)
                            self.publish_DistError(meas["Error"], t, meas["ModemID"], self.acousticError3)
                            self.publish_position(meas["ModemPos"], t, self.Anchor3)
                        self.dataWriter.writeCSVMeas()

                    elif counter == steps:
                        xest = self.localisation_sim.locate(preInput, t, depth, meas=None)
                        self.publish_position(xest, t, self.position_pub)
                        self.publish_PosError(x, t, xest, self.errAbs)
                      
                        counter = 1
                    else:
                        counter +=1
                r.sleep()

        elif not self.ros:
            time_gps, UTMPosx, UTMPosy, depth, vx, vy, vz, meas_data = self.dataloader.inputClearedData()
            
            for i in tqdm(range(len(time_gps)), ncols=100):
                x = np.array([UTMPosx[i], UTMPosy[i], depth[i]]).T
                self.x0 = x
                preInput = np.array([vx[i], vy[i], vz[i]]).T + np.random.normal(self.filter_config["config"][0]["MeasErrLoc"],self.filter_config["config"][0]["MeasErrScale"],3)
                meas = self.acoustic_sim.simulate(x, time_gps[i])
                self.dataWriter.fillState(x[0],x[1],x[2],time_gps[i])
                self.dataWriter.writeCSVState()
                if meas is not None:
                    err, zhat = self.measErrDist(meas["ModemPos"], meas["dist"], x)
                    self.dataWriter.fillMeas(meas["time_published"], meas["tr"], meas["dist"], meas["realDist"], meas["ModemID"], meas["Error"])
                    self.dataWriter.fillErr(err)
                    self.dataWriter.writeCSVMeas()
                    self.plot1.addMeas(meas["dist"], meas["Error"], meas["time_published"], meas["ModemID"]) #dist, time, ID
                    XFilter = self.localisation_sim.locate(preInput, time_gps[i], x[2], meas)
                    self.plot1.addPosFilter(time_gps[i], XFilter)
                elif meas is None:
                    XFilter = self.localisation_sim.locate(preInput, time_gps[i], x[2], meas=None)
                    self.plot1.addPosFilter(time_gps[i], XFilter)
                else:
                    print("[Simulation_Class]: Error in manual dataInput") 

                self.plot1.addPosGPS(time_gps[i], x)
            self.plot1.plot(meas_data)
            #self.plot1.plotPath()
            #self.plot1.plotMeas(meas_data)
            return

def main():
    simu = simulation()
    simu.run()

    plt.show()
    print("Finished")

if __name__ == "__main__":
    main()



