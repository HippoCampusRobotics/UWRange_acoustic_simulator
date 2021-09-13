{
    "config": [
        {
            "type": "setup",
            "PacketLengthPoll": 0.1, #[s]
            "PacketLengthResponse": 0.2, #[s]
            "SOS": 1450, #[m/s]
            "algorithm": "broadcast", #broadcast or alternating
            "pollcircle": "timetrgd", #timetrgd (timetriggered) or lstAcktrgd (time and last Poll triggert)
            "updateTime": 3, #[s] time of updateCircle
            "PrcTime": 0.3, #[s] Time Modem is Processing
            "PublishDelay": 0.05, #[s] delay caused by data connection
            "TimeOut": 0.2 #[s] Time Soundwave exists
            "numberAnchor": 4, 
            "FrequencyAcousticSim": 200, #[Hz] Frequenzy of acoustic simulation
            "MeasErrLoc": 0, # white noise Meas (location)
            "MeasErrScale": 0 # white noise Meas (scale)

        },
        {
            "type": "agent",
            "name": "AUV",
            "shortname": "AUV",
            "modem": {
                "id": 0,
                "DelayTime": 0,
                "PacketReceptionRate": 1, #should be 1
                "dst": "broadcast",
                "packetTyp": "TYPE_RANGING_POLL"
            },
            "position":[0,0,0]
        },
        {
            "type": "anchor",
            "name": "Anchor 1",
            "shortname": "anch_1",
            "modem": {
                "id": 1,
                "DelayTime": 0.7, #Delay befor sending Acknowlegdment
                "PacketReceptionRate": 1, #1 = 100%  0= 0%
                "dst": "broadcast", #Destiny
                "packetTyp": "TYPE_RANGING_ACK"
            },
            "position":[0,0,0]
        },
        {
            "type": "anchor",
            "name": "Anchor 2",
            "shortname": "anch_2",
            "modem": {
                "id": 2,
                "DelayTime": 1.4,
                "PacketReceptionRate": 1,
                "dst": "broadcast",
                "packetTyp": "TYPE_RANGING_ACK"
            },
            "position":[0,0,0]
        },
        {
            "type": "anchor",
            "name": "Anchor 3",
            "shortname": "anch_3",
            "modem": {
                "id": 3,
                "DelayTime": 2.1,
                "PacketReceptionRate": 1,
                "dst": "broadcast",
                "packetTyp": "TYPE_RANGING_ACK"
            },
            "position":[0,0,0]
        },
        {
            "type": "anchor",
            "name": "Anchor 4",
            "shortname": "anch_4",
            "modem": {
                "id": 4,
                "DelayTime": 2.8,
                "PacketReceptionRate": 1,
                "dst": "broadcast",
                "packetTyp": "TYPE_RANGING_ACK"
            },
            "position":[0,0,0]
        }
        
    ]
}