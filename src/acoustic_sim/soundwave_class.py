#!/usr/bin/env python
class soundwave_cl:
    def __init__(self, position, packet):
        self.radius = 0
        self.radius_t_1 = 0
        self.position = position
        self.packet = packet

    def update(self, dt, SOS):
        self.radius_t_1 = self.radius
        self.radius += dt * SOS
    
    def getRadius(self):
        return self.radius

    def getRadius_t_1(self):
        return self.radius_t_1

    def getPosition(self):
        return self.position

    def getPacket(self):
        return self.packet

    def setRadius(self, dt, SOS):
        self.radius = dt* SOS
        self.radius_t_1 = self.radius
