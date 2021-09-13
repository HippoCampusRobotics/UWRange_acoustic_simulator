#!/usr/bin/env python
import json
import os

class packet:
    def __init__(self, tx_time, tx_pos, type, src, dst, TimeDiff, length):
        
        tmp = os.path.dirname(__file__)
        file_path_filter = os.path.join(tmp, '../../config/acoustic_config.json')
        f = open(file_path_filter)
        self.acoustic_config = json.load(f)
        f.close()

        self.tx_time = tx_time
        self.tx_pos = tx_pos
        self.length = length
        self.anchorPrcTime = 0
        self.type = type
        self.src = src
        self.dst = dst
        self.timeout = self.acoustic_config["config"][0]["TimeOut"] + self.tx_time # timeout aus configfile        

    def getPacketDict(self):
        self.dict = {"tx_time": self.tx_time, "tx_pos": self.tx_pos, "type": self.type, "src": self.src, "dst": self.dst, "timeout": self.timeout, "length": self.length, "AnchorPrcTime": self.anchorPrcTime}
        return self.dict
    
    def setAnchorPrcTime(self, t):
        self.anchorPrcTime = t
    
