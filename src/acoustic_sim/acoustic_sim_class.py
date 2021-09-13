#!/usr/bin/env python

import numpy as np
from collections import deque
from .modem_class import modem
import json
import os


class acousticSimulation:
    def __init__(self):

        tmp = os.path.dirname(__file__)
        file_path_filter = os.path.join(tmp, '../../config/acoustic_config.json')
        f = open(file_path_filter)
        self.config = json.load(f)
        f.close()

        self.t = 0
        self.last_t = 0
        self.dt = 0
        self.soundwaveList = deque([])
        self.AgentModemList = self.fillAgentModemList()
        self.AnchorModemList = self.fillAnchorModemList()
        if self.config["config"][0]["algorithm"] == "broadcast":
            self.AgentDst = "broadcast"
            self.AnchorDst = "broadcast"
        elif self.config["config"][0]["algorithm"] == "alternating":
            self.AgentDst = self.AnchorIdList[0]
            self.AnchorDst = self.AgentIdList[0]
        else: print("[Acousitc-sim] Dst Error")
        self.AgentModemList[0].sendPoll(self.AgentDst) #Agent sends first Poll
        self.DstCounter = 0
        self.TimeOutAlternating = self.config["config"][0]["TimeOutAlternating"]

        
        
    def simulate(self, x, t):
        self.t = float(t)
        self.dt = float(self.t - self.last_t)
        self.last_t = self.t
        
        self.updateSoundwave(self.dt)
        self.deletSoundwave()
        self.updateAgentModem(x)
        self.updateAnchorModem()
        return self.published()    

    def updateSoundwave(self, dt):
        SOS = self.getSOS()
        if not len(self.soundwaveList) == 0:
            for soundwave in self.soundwaveList:
                soundwave.update(dt, SOS)

    def updateAgentModem(self, x):
        for modem in self.AgentModemList:
            ret = modem.update(x, self.soundwaveList, self.t, self.getSOS(), self.AgentDst)   
            if ret is not None:
                self.soundwaveList.append(ret)

    def updateAnchorModem(self):
        for modem in self.AnchorModemList:
            ret = modem.update(modem.getPosition(), self.soundwaveList, self.t, self.getSOS(), self.AnchorDst)
            if ret is not None:
                self.soundwaveList.append(ret)

    def published(self):
        if self.DstCounter <= self.t:
            self.newDst(self.AgentDst)

        meas = None
        for modem in self.AgentModemList:
            if modem.getPublished():
                meas = modem.getPublishedMessage()
                modem.setPublishedFlag(False) 
                self.newDst(meas["ModemID"])
                self.setDstCounter()
        return meas

    def deletSoundwave(self):
        counter = 0
        for soundwave in self.soundwaveList:
            if soundwave.getPacket().getPacketDict()["timeout"] <= self.t:
                counter += 1
        
        for i in range(counter):
            self.soundwaveList.popleft()
    
    def setDstCounter(self):
        self.DstCounter = self.t + self.TimeOutAlternating
    
    def newDst(self, ID):
        if self.config["config"][0]["algorithm"] == "broadcast":
            self.AgentDst = "broadcast"
            self.AnchorDst = "broadcast"
        elif self.config["config"][0]["algorithm"] == "alternating":
            for i in range(len(self.AnchorIdList)):
                if self.AnchorIdList[i] == ID:
                    if i < len(self.AnchorIdList)-1:
                        self.AgentDst = self.AnchorIdList[i+1]
                    elif i >= len(self.AnchorIdList)-1:
                        self.AgentDst = self.AnchorIdList[0]

    def fillAnchorModemList(self): 
        Anchorlist = []
        self.AnchorIdList = []
        for i in self.config["config"]:
            if i['type'] == 'anchor':
                creatmodem = modem(i["type"], i["name"], i["position"], i["modem"]["id"], 
                i["modem"]["DelayTime"], i["modem"]["PacketReceptionRate"], i["modem"]["dst"], 
                i["modem"]["packetTyp"])
                idmodem = i["modem"]["id"]
                Anchorlist.append(creatmodem)
                self.AnchorIdList.append(idmodem)
        return Anchorlist
    
    def fillAgentModemList(self): 
        Agentlist = []
        self.AgentIdList = []
        for i in self.config["config"]:
            if i["type"]=="agent":
                creatmodem = modem(i["type"], i["name"], i["position"], i["modem"]["id"], 
                i["modem"]["DelayTime"], i["modem"]["PacketReceptionRate"], i["modem"]["dst"], 
                i["modem"]["packetTyp"])
                idmodem = i["modem"]["id"]
                self.AgentIdList.append(idmodem)
                Agentlist.append(creatmodem)
        return Agentlist
    
    def getSOS(self):
        return self.config["config"][0]["SOS"]
    
    def getCounter(self):
        for modem in self.AgentModemList:
            print(modem.getSWCounter())
        
        for modem in self.AnchorModemList:
            print(modem.getSWCounter())

def main():
    sim = acousticSimulation()

if __name__ == "__main__":
        main()
