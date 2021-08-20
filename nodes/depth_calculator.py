#!/usr/bin/env python

import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32
import os
import json



def pressure_callback(pressure_msg, publisher):
   pascal_per_meter = 1.0e4
   airpressure = 1.0e5
   offset = 0.13 #[m]
   depth = offset - (pressure_msg.fluid_pressure- airpressure) / pascal_per_meter 
   depth_msg = Float32()
   depth_msg.data = depth
   publisher.publish(depth_msg)


def main():
   tmp = os.path.dirname(__file__)
   file_path_filter = os.path.join(tmp, '../config/acoustic_config.json')
   f = open(file_path_filter)
   acoustic_config = json.load(f)
   f.close()


   rospy.init_node("depth_calculator")
   depth_pub = rospy.Publisher("depth", Float32, queue_size=1)
   if acoustic_config["config"][0]["SimulationPath"]:
      pressure_sub = rospy.Subscriber("/bluero/pressure", FluidPressure, pressure_callback, depth_pub)
   else:
      pressure_sub = rospy.Subscriber("pressure", FluidPressure, pressure_callback, depth_pub)
   rospy.spin()


if __name__ == "__main__":
   main()
