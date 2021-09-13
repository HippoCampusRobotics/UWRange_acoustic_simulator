#!/usr/bin/env python
from .packet_class import packet
from .soundwave_class import soundwave_cl
import numpy as np
import json
import os

class modem:
    def __init__(self, typ, name, position, ID, DelayTime, packetReceptionRate, dst, packetType):       
        tmp = os.path.dirname(__file__)
        file_path_filter = os.path.join(tmp, '../../config/acoustic_config.json')
        f = open(file_path_filter)
        self.config = json.load(f)
        f.close()
        self.state = "IDLE"
        
        self.role = typ
        self.packetTyp = packetType
        self.name = name
        self.position = position
        self.modemID = ID
        self.T_wp = self.config["config"][0]["T_wp"]
        self.T_wr = self.config["config"][0]["T_wr"]
        self.pollcircle = self.config["config"][0]["pollcircle"]
        self.packetReceptionRate = packetReceptionRate
        self.packetLengthPoll = self.config["config"][0]["PacketLengthPoll"]
        self.packetLengthResponse = self.config["config"][0]["PacketLengthResponse"]
        self.publishDelay = self.config["config"][0]["PublishDelay"]
        self.PollCircleTime = self.config["config"][0]["PollCircleTime"]
        self.TimeOutAlternating = self.config["config"][0]["TimeOutAlternating"]
        self.numberAnchor = self.config["config"][0]["numberAnchor"]
        self.algorithm = self.config["config"][0]["algorithm"]
        
        if self.algorithm == "alternating":
            self.delay = 0
        else:
            self.delay = DelayTime       
        
        self.dst =  dst
        self.SOS = self.config["config"][0]["SOS"] #[m/s] 

        self.sim_time = 0
        self.last_sim_time = 0
        
        self.soundwaveList = []
        self.last_soundwaveList = []
        self.receivingList = []
        self.receivingTimeList = []
        self.last_poll = 0
        self.next_poll = self.last_poll + self.PollCircleTime
        self.inRange = False

        self.transmitEndTime = None
        self.packet = None
        self.soundwave = None
        self.receivedSoundwave = None
        self.receivingTime = 0
        
        
        self.publishFlag = False
        self.publishedMessage = None
        self.last_position = position
        self.transmitPrcTime = 0
        self.AnchorPrcTime = self.T_wp + self.delay
        self.AckCounter = 0
      
        self.PollPermitted = False
        

        #debugging
        self.swCounter = 0
       

    def update(self, position, soundwaveList, sim_time, SOS, dst):
        self.updateDst(dst)
        self.updatePosition(position)
        self.updateSimTime(sim_time)
        self.updateSoundwaveList(soundwaveList)
        self.updateSOS(SOS)
        ret = None
        
        if self.state == "IDLE":
            if self.algorithm == "broadcast":
                if self.pollcircle == "timetrgd":
                    if self.role == "agent" and self.next_poll <= self.sim_time:
                        self.state = "DELAY"
                        self.resetAckCounter()
                        self.delayTime = self.sim_time + self.T_wr        

                elif self.pollcircle == "lstAcktrgd":
                    if self.role == "agent" and (self.next_poll <= self.sim_time or self.AckCounter >= self.numberAnchor):
                        self.resetAckCounter()
                        self.state = "DELAY"
                        self.delayTime = self.sim_time + self.T_wr   
                
                else:
                    print("[Modem] Wrong Pollcircle!")
                    
            
            elif self.algorithm == "alternating":  
                if self.pollcircle == "timetrgd":
                    if self.role == "agent" and self.next_poll <= self.sim_time:
                        self.state = "DELAY"
                        self.delayTime = self.sim_time + self.T_wr
                        self.PollPermitted = False
                    
                elif self.pollcircle == "lstAcktrgd":
                    if self.role == "agent" and (self.PollPermitted or self.next_poll <= self.sim_time):
                        self.state = "DELAY"
                        self.delayTime = self.sim_time + self.T_wr
                        self.PollPermitted = False

                else:
                    print("[Modem] Wrong Pollcircle!")
            else:
                print("[Modem] Wrong algorithm")

            self.resetReceivingList()
            for soundwave in soundwaveList:
                
                if self.isInRange(soundwave) and self.state == "IDLE":
                    self.receivedPacket = soundwave.getPacket().getPacketDict()
                    self.receivingTime = self.interpolateReceivingTime(soundwave)
                    self.receivingTimeList.append(self.receivingTime)
                    self.receivingList.append(self.receivedPacket)
            
            if self.receivingList:
                index = min(range(len(self.receivingTimeList)), key=self.receivingTimeList.__getitem__)
                self.receivedPacket = self.receivingList[index]
                self.receivingTime = self.receivingTimeList[index]
                self.state = "RECEIVE"
                self.exittime = self.receivingTime + self.receivedPacket["length"]

        if self.state == "RECEIVE":
            if self.exittime <= self.sim_time:
                if self.receivedPacket["type"] == "TYPE_RANGING_POLL" and self.role =="anchor" and (self.receivedPacket["dst"] == self.modemID or self.receivedPacket["dst"] == "broadcast"):
                        if self.packetLost():
                            self.state = "IDLE"
                        else:
                            self.state = "DELAY"
                            self.delayTime = self.exittime + self.delay + self.T_wp 
                
                elif self.receivedPacket["type"] == "TYPE_RANGING_ACK" and self.role == "agent" and (self.receivedPacket["dst"] == self.modemID or self.receivedPacket["dst"] == "broadcast"):
                    self.runtime = float(self.receivingTime - self.last_poll - self.receivedPacket["AnchorPrcTime"] ) # AnchorPrcTime = delay + PrcTime + Zeitausgleich durch diskrete iteration (schallwelle)
                    
                    self.SOS = self.getSOS()
                    dist = (self.runtime/2) * self.SOS + float(np.random.normal(self.config["config"][0]["MeasErrLoc"], self.config["config"][0]["MeasErrScale"],1))
                    exittime = self.exittime + self.publishDelay
                    if exittime <= self.sim_time:
                        if self.packetLost():
                            self.state = "IDLE"
                        else:
                            self.publish(dist, self.exittime, self.receivedPacket["src"], self.receivedPacket["tx_pos"], self.packetLengthResponse, self.packetLengthPoll) # exittime - packetLengthResponse - publishDelay = True meas Time
                            self.PollPermitted = True
                            self.AckCounter += 1 
                            self.state = "IDLE"
                else:
                    self.state = "IDLE"

        if self.state == "DELAY":
            if self.delayTime <= self.sim_time:
                self.state = "TRANSMIT"
                
                if self.role == "agent":
                    self.packet = packet(self.sim_time, self.position, self.packetTyp, self.modemID, self.dst, 0, self.packetLengthPoll)
                    self.soundwave = soundwave_cl(self.position, self.packet)
                    self.transmitEndTime = self.sim_time + self.packetLengthPoll
                    self.last_poll = float(self.sim_time)
                    if self.algorithm == "broadcast":
                        self.next_poll = self.last_poll + self.PollCircleTime
                    elif self.algorithm == "alternating":
                        self.next_poll = self.last_poll + self.TimeOutAlternating
                    ret = self.soundwave
                    self.swCounter += 1

                if self.role == "anchor":
                    self.transmitEndTime = self.sim_time + self.packetLengthResponse
                    self.packet = packet(self.sim_time, self.position, self.packetTyp, self.modemID, self.dst, 0, self.packetLengthResponse) 
                    self.packet.setAnchorPrcTime(self.sim_time- self.receivingTime)
                    self.soundwave = soundwave_cl(self.position, self.packet)
                    ret = self.soundwave
                    self.swCounter += 1

        if self.state == "TRANSMIT":              
            if self.transmitEndTime <= self.sim_time:
                    self.state = "IDLE"

        return ret #new Packet for soundwaveList

    def isInRange(self, soundwave):
        radiusSoundwave = soundwave.getRadius()
        rangeReceiver = np.absolute(np.linalg.norm(np.array(self.position- np.array(soundwave.getPosition()))))

        if radiusSoundwave >= rangeReceiver:
            self.inRange = True
            return True
        
        return False

    def interpolateReceivingTime(self, soundwave):
        rangeReceiver = np.absolute(np.linalg.norm(np.array(self.position - np.array(soundwave.getPosition()))))
        rangeReceiver_t_1 = np.absolute(np.linalg.norm(np.array(self.last_position - np.array(soundwave.getPosition()))))
        dRangeReceiver = rangeReceiver - rangeReceiver_t_1
        dRangeSoundwave = soundwave.getRadius() - soundwave.getRadius_t_1()  
        
        if self.inRange == True: # if soundwave reached receiver and soundwave t-1 has not rechead receiver, calculate intersection with the help of interpolation
            self.t_runtime = self.last_sim_time + (self.sim_time - self.last_sim_time) * (soundwave.getRadius_t_1() -  rangeReceiver_t_1) / (dRangeReceiver - dRangeSoundwave) # "true" runtime soundwave
            self.sim_time_last_ack = self.sim_time
            self.inRange = False
            return self.t_runtime
 
    def publish(self, dist, exittime, ID, position, PacketLengthPoll, PacketLengthResponse):
        self.publishFlag = True
        self.publishedMessage = {"dist": dist, "time_published": exittime, "ModemID": ID, "ModemPos": position, "PacketLengthPoll":PacketLengthPoll, "PacketLengthResponse":PacketLengthResponse}

    def getSOS(self):
        return self.SOS

    def getPosition(self):
        return self.position
    
    def resetAckCounter(self):
        self.AckCounter = 0
    
    def resetReceivingList(self):
        self.receivingList = []
        self.receivingTimeList = []

    def packetLost(self):
        if np.random.binomial(1,(self.packetReceptionRate)) == 1:
            return False
        return True

    def updateDst(self, dst):
        self.dst = dst 

    def updateSOS(self, SOS):
        self.SOS = SOS
    
    def updatePosition(self, position):
        self.last_position = self.position
        self.position = position
    
    def updateSimTime(self, sim_time):
        self.last_sim_time = self.sim_time
        self.sim_time = sim_time
    
    def updateSoundwaveList(self, soundwaveList):
        self.last_soundwaveList = self.soundwaveList
        self.soundwaveList = soundwaveList

    def setPublishedFlag(self, bool):
        self.publishFlag= bool

    def sendPoll(self, dst):
        self.delayTime = self.sim_time 
        self.updateDst(dst)
        self.state = "DELAY"
        
    def getPublished(self):
        return self.publishFlag
    
    def getPublishedMessage(self):
        return self.publishedMessage
    
    def getSoundwaves(self):
        return self.soundwaveList
    
    def getSWCounter(self):
        return self.name, self.swCounter 
   
