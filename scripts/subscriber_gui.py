#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dvl_a50_ros_driver.msg import DVL
from dvl_a50_ros_driver.msg import DVLBeam
import tkinter as tk
import threading


class DVLA50SubscriberGUI(Node):
	def __init__(self, root):
		super().__init__('dvl_a50_subscriber_gui')
		self.root = root
		
		# Global variables to handle all the parsed JSON data
		self.gTime = tk.DoubleVar(root)
		self.gVX = tk.DoubleVar(root)
		self.gVY = tk.DoubleVar(root)
		self.gVZ = tk.DoubleVar(root)
		self.gFom = tk.DoubleVar(root)
		self.gAltitude = tk.DoubleVar(root)
		self.gVelValid = tk.StringVar(root)
		self.gStatus = tk.StringVar(root)
		self.gForm = tk.StringVar(root)
		
		self.g0ID = tk.StringVar(root)
		self.g0Vel = tk.DoubleVar(root)
		self.g0Distance = tk.DoubleVar(root)
		self.g0rssi = tk.DoubleVar(root)
		self.g0nsd = tk.DoubleVar(root)
		self.g0valid = tk.StringVar(root)
		
		self.g1ID = tk.StringVar(root)
		self.g1Vel = tk.DoubleVar(root)
		self.g1Distance = tk.DoubleVar(root)
		self.g1rssi = tk.DoubleVar(root)
		self.g1nsd = tk.DoubleVar(root)
		self.g1valid = tk.StringVar(root)
		
		self.g2ID = tk.StringVar(root)
		self.g2Vel = tk.DoubleVar(root)
		self.g2Distance = tk.DoubleVar(root)
		self.g2rssi = tk.DoubleVar(root)
		self.g2nsd = tk.DoubleVar(root)
		self.g2valid = tk.StringVar(root)
		
		self.g3ID = tk.StringVar(root)
		self.g3Vel = tk.DoubleVar(root)
		self.g3Distance = tk.DoubleVar(root)
		self.g3rssi = tk.DoubleVar(root)
		self.g3nsd = tk.DoubleVar(root)
		self.g3valid = tk.StringVar(root)
		
		# Create subscriptions
		self.raw_subscription = self.create_subscription(
			String,
			'dvl/json_data',
			self.callbackRAW,
			10)
		self.dvl_subscription = self.create_subscription(
			DVL,
			'dvl/data',
			self.callback,
			10)
	
	def callbackRAW(self, data):
		self.get_logger().info(f'Data received: {data.data}')
	
	def callback(self, data):
		self.gTime.set('%2.2f' % data.time)
		self.gVX.set('%2.2f' % data.velocity.x)
		self.gVY.set('%2.2f' % data.velocity.y)
		self.gVZ.set('%2.2f' % data.velocity.z)
		self.gFom.set('%2.2f' % data.fom)
		self.gAltitude.set('%2.2f' % data.altitude)
		self.gVelValid.set(str(data.velocity_valid))
		self.gStatus.set(str(data.status))
		self.gForm.set(str(data.form))
		
		self.g0ID.set(str(data.beams[0].id))
		self.g0Vel.set(str(data.beams[0].velocity))
		self.g0Distance.set(str(data.beams[0].distance))
		self.g0rssi.set(str(data.beams[0].rssi))
		self.g0nsd.set(str(data.beams[0].nsd))
		self.g0valid.set(str(data.beams[0].valid))
		
		self.g1ID.set(str(data.beams[1].id))
		self.g1Vel.set(str(data.beams[1].velocity))
		self.g1Distance.set(str(data.beams[1].distance))
		self.g1rssi.set(str(data.beams[1].rssi))
		self.g1nsd.set(str(data.beams[1].nsd))
		self.g1valid.set(str(data.beams[1].valid))
		
		self.g2ID.set(str(data.beams[2].id))
		self.g2Vel.set(str(data.beams[2].velocity))
		self.g2Distance.set(str(data.beams[2].distance))
		self.g2rssi.set(str(data.beams[2].rssi))
		self.g2nsd.set(str(data.beams[2].nsd))
		self.g2valid.set(str(data.beams[2].valid))
		
		self.g3ID.set(str(data.beams[3].id))
		self.g3Vel.set(str(data.beams[3].velocity))
		self.g3Distance.set(str(data.beams[3].distance))
		self.g3rssi.set(str(data.beams[3].rssi))
		self.g3nsd.set(str(data.beams[3].nsd))
		self.g3valid.set(str(data.beams[3].valid))


def setup_gui(root, subscriber_node):
	"""Setup GUI layout and design"""
	
	root.title("Waterlinked DVL A50")
	
	label = tk.Label(root, text="Data collected from the DVL")
	label.grid(columnspan=5, sticky=tk.W)
	label.config(font=("Courier", 26))
	
	label_ph1 = tk.Label(root,  text="            ")
	label_ph1.grid(row=1, column=1)
	label_ph1.config(font=("Courier", 22))
	label_ph2 = tk.Label(root,  text="            ")
	label_ph2.grid(row=1, column=2)
	label_ph2.config(font=("Courier", 22))
	label_ph3 = tk.Label(root,  text="            ")
	label_ph3.grid(row=1, column=3)
	label_ph3.config(font=("Courier", 22))
	label_ph4 = tk.Label(root,  text="            ")
	label_ph4.grid(row=1, column=4)
	label_ph4.config(font=("Courier", 22))
	
	label_timeT = tk.Label(root,  text="Time:")
	label_timeT.grid(row=3, sticky=tk.E)
	label_timeT.config(font=("Courier", 22))
	
	label_time = tk.Label(root,  textvariable=subscriber_node.gTime)
	label_time.grid(row=3, column=1)
	label_time.config(font=("Courier", 22))
	
	label_velT = tk.Label(root,  text="Velocity:")
	label_velT.grid(row=4, sticky=tk.E)
	label_velT.config(font=("Courier", 22))
	
	label_velX = tk.Label(root,  textvariable=subscriber_node.gVX)
	label_velX.grid(row=4, column=1)
	label_velX.config(font=("Courier", 22))
	label_velY = tk.Label(root,  textvariable=subscriber_node.gVY)
	label_velY.grid(row=4, column=2)
	label_velY.config(font=("Courier", 22))
	label_velZ = tk.Label(root,  textvariable=subscriber_node.gVZ)
	label_velZ.grid(row=4, column=3)
	label_velZ.config(font=("Courier", 22))
	
	label_fomT = tk.Label(root,  text="FOM:")
	label_fomT.grid(row=5, sticky=tk.E)
	label_fomT.config(font=("Courier", 22))
	
	label_fom = tk.Label(root,  textvariable=subscriber_node.gFom)
	label_fom.grid(row=5, column=1)
	label_fom.config(font=("Courier", 22))
	
	label_altT = tk.Label(root,  text="Altitude:")
	label_altT.grid(row=6, sticky=tk.E)
	label_altT.config(font=("Courier", 22))
	
	label_alt = tk.Label(root,  textvariable=subscriber_node.gAltitude)
	label_alt.grid(row=6, column=1)
	label_alt.config(font=("Courier", 22))
	
	label_valT = tk.Label(root,  text="Velocity Valid:")
	label_valT.grid(row=7, sticky=tk.E)
	label_valT.config(font=("Courier", 22))
	
	label_val = tk.Label(root,  textvariable=subscriber_node.gVelValid)
	label_val.grid(row=7, column=1)
	label_val.config(font=("Courier", 22))
	
	label_transT = tk.Label(root, text="Transducers:")
	label_transT.grid(row=8, sticky=tk.E)
	label_transT.config(font=("Courier", 22))
	label_beam0T = tk.Label(root, text="Beam:0", bg="gray80")
	label_beam0T.grid(row=8, column=1, sticky='ew')
	label_beam0T.config(font=("Courier", 22))
	label_beam1T = tk.Label(root, text="Beam:1", bg="gray100")
	label_beam1T.grid(row=8, column=2, sticky='ew')
	label_beam1T.config(font=("Courier", 22))
	label_beam2T = tk.Label(root, text="Beam:2", bg="gray80")
	label_beam2T.grid(row=8, column=3, sticky='ew')
	label_beam2T.config(font=("Courier", 22))
	label_beam3T = tk.Label(root, text="Beam:3", bg="gray100")
	label_beam3T.grid(row=8, column=4, sticky='ew')
	label_beam3T.config(font=("Courier", 22))
	
	label_beamVelT = tk.Label(root, text="Velocity")
	label_beamVelT.grid(row=9, sticky=tk.E)
	label_beamVelT.config(font=("Courier", 15))
	label_beamVel0 = tk.Label(root, textvariable=subscriber_node.g0Vel, bg="gray75")
	label_beamVel0.grid(row=9, column=1, sticky='ew')
	label_beamVel0.config(font=("Courier", 15))
	label_beamVel1 = tk.Label(root, textvariable=subscriber_node.g1Vel, bg="gray95")
	label_beamVel1.grid(row=9, column=2, sticky='ew')
	label_beamVel1.config(font=("Courier", 15))
	label_beamVel2 = tk.Label(root, textvariable=subscriber_node.g2Vel, bg="gray75")
	label_beamVel2.grid(row=9, column=3, sticky='ew')
	label_beamVel2.config(font=("Courier", 15))
	label_beamVel3 = tk.Label(root, textvariable=subscriber_node.g3Vel, bg="gray95")
	label_beamVel3.grid(row=9, column=4, sticky='ew')
	label_beamVel3.config(font=("Courier", 15))
	
	label_beamDistT = tk.Label(root, text="Distance")
	label_beamDistT.grid(row=10, sticky=tk.E)
	label_beamDistT.config(font=("Courier", 15))
	label_beamDist0 = tk.Label(root, textvariable=subscriber_node.g0Distance, bg="gray80")
	label_beamDist0.grid(row=10, column=1, sticky='ew')
	label_beamDist0.config(font=("Courier", 15))
	label_beamDist1 = tk.Label(root, textvariable=subscriber_node.g1Distance, bg="gray100")
	label_beamDist1.grid(row=10, column=2, sticky='ew')
	label_beamDist1.config(font=("Courier", 15))
	label_beamDist2 = tk.Label(root, textvariable=subscriber_node.g2Distance, bg="gray80")
	label_beamDist2.grid(row=10, column=3, sticky='ew')
	label_beamDist2.config(font=("Courier", 15))
	label_beamDist3 = tk.Label(root, textvariable=subscriber_node.g3Distance, bg="gray100")
	label_beamDist3.grid(row=10, column=4, sticky='ew')
	label_beamDist3.config(font=("Courier", 15))
	
	label_beamRssiT = tk.Label(root, text="rssi")
	label_beamRssiT.grid(row=11, sticky=tk.E)
	label_beamRssiT.config(font=("Courier", 15))
	label_beamRssi0 = tk.Label(root, textvariable=subscriber_node.g0rssi, bg="gray75")
	label_beamRssi0.grid(row=11, column=1, sticky='ew')
	label_beamRssi0.config(font=("Courier", 15))
	label_beamRssi1 = tk.Label(root, textvariable=subscriber_node.g1rssi, bg="gray95")
	label_beamRssi1.grid(row=11, column=2, sticky='ew')
	label_beamRssi1.config(font=("Courier", 15))
	label_beamRssi2 = tk.Label(root, textvariable=subscriber_node.g2rssi, bg="gray75")
	label_beamRssi2.grid(row=11, column=3, sticky='ew')
	label_beamRssi2.config(font=("Courier", 15))
	label_beamRssi3 = tk.Label(root, textvariable=subscriber_node.g3rssi, bg="gray95")
	label_beamRssi3.grid(row=11, column=4, sticky='ew')
	label_beamRssi3.config(font=("Courier", 15))
	
	label_beamNsdT = tk.Label(root, text="nsd")
	label_beamNsdT.grid(row=12, sticky=tk.E)
	label_beamNsdT.config(font=("Courier", 15))
	label_beamNsd0 = tk.Label(root, textvariable=subscriber_node.g0nsd, bg="gray80")
	label_beamNsd0.grid(row=12, column=1, sticky='ew')
	label_beamNsd0.config(font=("Courier", 15))
	label_beamNsd1 = tk.Label(root, textvariable=subscriber_node.g1nsd, bg="gray100")
	label_beamNsd1.grid(row=12, column=2, sticky='ew')
	label_beamNsd1.config(font=("Courier", 15))
	label_beamNsd2 = tk.Label(root, textvariable=subscriber_node.g2nsd, bg="gray80")
	label_beamNsd2.grid(row=12, column=3, sticky='ew')
	label_beamNsd2.config(font=("Courier", 15))
	label_beamNsd3 = tk.Label(root, textvariable=subscriber_node.g3nsd, bg="gray100")
	label_beamNsd3.grid(row=12, column=4, sticky='ew')
	label_beamNsd3.config(font=("Courier", 15))
	
	label_beamValidT = tk.Label(root, text="Beam Valid")
	label_beamValidT.grid(row=13, sticky=tk.E)
	label_beamValidT.config(font=("Courier", 15))
	label_beamValid0 = tk.Label(root, textvariable=subscriber_node.g0valid, bg="gray75")
	label_beamValid0.grid(row=13, column=1, sticky='ew')
	label_beamValid0.config(font=("Courier", 15))
	label_beamValid1 = tk.Label(root, textvariable=subscriber_node.g1valid, bg="gray95")
	label_beamValid1.grid(row=13, column=2, sticky='ew')
	label_beamValid1.config(font=("Courier", 15))
	label_beamValid2 = tk.Label(root, textvariable=subscriber_node.g2valid, bg="gray75")
	label_beamValid2.grid(row=13, column=3, sticky='ew')
	label_beamValid2.config(font=("Courier", 15))
	label_beamValid3 = tk.Label(root, textvariable=subscriber_node.g3valid, bg="gray95")
	label_beamValid3.grid(row=13, column=4, sticky='ew')
	label_beamValid3.config(font=("Courier", 15))



def spin_ros(node):
	"""Spin ROS2 node in a separate thread"""
	rclpy.spin(node)


def main(args=None):
	rclpy.init(args=args)
	
	# GUI Setup
	root = tk.Tk()
	root.geometry("1150x450")
	
	# Create subscriber node
	subscriber_node = DVLA50SubscriberGUI(root)
	
	# Setup GUI
	setup_gui(root, subscriber_node)
	
	# Start ROS2 spinning in a separate thread
	ros_thread = threading.Thread(target=spin_ros, args=(subscriber_node,), daemon=True)
	ros_thread.start()
	
	# Handle window close event
	def on_closing():
		subscriber_node.destroy_node()
		rclpy.shutdown()
		root.destroy()
	
	root.protocol("WM_DELETE_WINDOW", on_closing)
	
	# Start Tkinter main loop
	root.mainloop()


if __name__ == '__main__':
	main()
