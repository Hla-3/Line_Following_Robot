#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math
import os

# 1. Setup paths for the custom Plotter and VSI API
current_dir = os.getcwd()
sys.path.append(current_dir)

# Ensure the script looks in the correct src folder if launched from root
sys.path.append(os.path.join(current_dir, 'src/Plotter'))

from realTimePlotter import RealTimePlotter

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

class MySignals:
	def __init__(self):
		# Inputs
		self.cmd_vel = [0] * 2
		self.robot_pose = [0] * 3
		self.reference_path = [0] * 3

# Start of user custom code region: Global Variables & Definitions
# End of user custom code region

class Plotter:
	def __init__(self, args):
		self.componentId = 2
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50103
		
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
		
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()

		# Start of user custom code region: Constructor
		self.plotter = RealTimePlotter(update_frequency=1)
		# End of user custom code region

	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiCanPythonGateway.initialize(dSession, self.componentId)
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region: After Reset
			# End of user custom code region
			
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region: Inside the while loop
			     if any(self.mySignals.robot_pose):  # Only plot if we have non-zero data
				self.plotter.update_data(sim_time_sec, self.mySignals.robot_pose, ...)
				# Update the Plotter UI with current data
				sim_time_sec = vsiCommonPythonApi.getSimulationTimeInNs() * 1e-9
				self.plotter.update_data(
					sim_time_sec, 
					self.mySignals.robot_pose, 
					self.mySignals.reference_path, 
					self.mySignals.cmd_vel
				)
				# End of user custom code region

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				# Receive Control Commands (ID 10)
				signalNumBytes = 16
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 10)
				self.mySignals.cmd_vel, receivedData = self.unpackBytes('d', receivedData, self.mySignals.cmd_vel)

				# Receive Robot Pose (ID 11)
				signalNumBytes = 24
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 11)
				self.mySignals.robot_pose, receivedData = self.unpackBytes('d', receivedData, self.mySignals.robot_pose)

				# Receive Reference Path (ID 12)
				signalNumBytes = 24
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 12)
				self.mySignals.reference_path, receivedData = self.unpackBytes('d', receivedData, self.mySignals.reference_path)

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")
				
				nextExpectedTime += self.simulationStep

				if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
					continue

				if(nextExpectedTime > self.totalSimulationTime):
					remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
					vsiCommonPythonApi.advanceSimulation(remainingTime)
					break

				vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())
		except Exception as e:
			if str(e) == "stopRequested":
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print("An error occurred: {}".format(str(e)))
		except:
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)

	def packBytes(self, signalType, signal):
		if isinstance(signal, list):
			return struct.pack('={}{}'.format(len(signal), signalType), *signal)
		return struct.pack('={}'.format(signalType), signal)

	def unpackBytes(self, signalType, packedBytes, signal = ""):
		if isinstance(signal, list):
			size = struct.calcsize('={}'.format(signalType))
			unpackedVariable = struct.unpack('={}{}'.format(len(signal), signalType), packedBytes[:len(signal)*size])
			return list(unpackedVariable), packedBytes[len(unpackedVariable)*size:]
		else:
			numBytes = struct.calcsize('={}'.format(signalType))
			unpackedVariable = struct.unpack('={}'.format(signalType), packedBytes[0:numBytes])[0]
			return unpackedVariable, packedBytes[numBytes:]

	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested()
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()

def main():
	inputArgs = argparse.ArgumentParser()
	inputArgs.add_argument('--domain', default='AF_UNIX')
	inputArgs.add_argument('--server-url', default='localhost')
	args = inputArgs.parse_args()
	
	plotter_inst = Plotter(args)
	plotter_inst.mainThread()

if __name__ == '__main__':
	main()