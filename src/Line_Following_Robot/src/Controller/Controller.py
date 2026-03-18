#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math
import os

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

class MySignals:
    def __init__(self):
        # Inputs
        self.robot_pose = [0] * 3
        self.reference_path = [0] * 3
        # Outputs
        self.cmd_vel = [0] * 2

# Start of user custom code region: Global Variables & Definitions
# These are used for the line-following PID logic
# End of user custom code region

class Controller:
    def __init__(self, args):
        self.componentId = 1
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50102
        
        self.simulationStep = 0
        self.stopRequested = False
        self.totalSimulationTime = 0
        
        self.receivedNumberOfBytes = 0
        self.receivedPayload = []
        self.mySignals = MySignals()

        # Start of user custom code region: Constructor
        self.kp = 1.2
        self.kd = 0.1
        self.prev_error = 0.0
        # End of user custom code region

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        try:
            vsiCommonPythonApi.waitForReset()

            # Start of user custom code region: After Reset
            nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
            # End of user custom code region
            
            self.updateInternalVariables()

            if(vsiCommonPythonApi.isStopRequested()):
                raise Exception("stopRequested")
            
            nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
            while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

                # Start of user custom code region: Inside the while loop
                # Calculate lateral error (Reference Y - Actual Y)
                lateral_error = self.mySignals.reference_path[1] - self.mySignals.robot_pose[1]
                
                # Effective dt is simulationStep * 10 (converted from ns to seconds)
                dt_seconds = (self.simulationStep * 10) * 1e-9 
                
                if dt_seconds > 0:
                    derivative = (lateral_error - self.prev_error) / dt_seconds
                else:
                    derivative = 0
                
                v_cmd = 0.5 
                omega_cmd = (self.kp * lateral_error) + (self.kd * derivative)
                
                self.mySignals.cmd_vel = [v_cmd, omega_cmd]
                self.prev_error = lateral_error
                # End of user custom code region

                self.updateInternalVariables()

                if(vsiCommonPythonApi.isStopRequested()):
                    raise Exception("stopRequested")

                # Receive robot_pose (ID 11) and reference_path (ID 12)
                signalNumBytes = 24
                receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 11)
                self.mySignals.robot_pose, receivedData = self.unpackBytes('d', receivedData, self.mySignals.robot_pose)

                signalNumBytes = 24
                receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 12)
                self.mySignals.reference_path, receivedData = self.unpackBytes('d', receivedData, self.mySignals.reference_path)

                # Send cmd_vel (ID 10)
                vsiCanPythonGateway.setCanId(10)
                vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.cmd_vel), 0, 64)
                vsiCanPythonGateway.setDataLengthInBits(64)
                vsiCanPythonGateway.sendCanPacket()

                print("\n+=Controller (100Hz) =+")
                print("  VSI time: {} ns".format(vsiCommonPythonApi.getSimulationTimeInNs()))
                print("  Inputs:  Pose={}, Path={}".format(self.mySignals.robot_pose, self.mySignals.reference_path))
                print("  Outputs: Cmd_Vel={}".format(self.mySignals.cmd_vel))

                self.updateInternalVariables()

                if(vsiCommonPythonApi.isStopRequested()):
                    raise Exception("stopRequested")
                
                # Frequency logic: Run at 100Hz by skipping 10 steps (if global step is 1ms) [cite: 1, 106]
                nextExpectedTime += self.simulationStep * 10 

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
                print(f"An error occurred: {str(e)}")
        except:
            vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)

    def packBytes(self, signalType, signal):
        if isinstance(signal, list):
            return struct.pack(f'={len(signal)}{signalType}', *signal)
        return struct.pack(f'={signalType}', signal)

    def unpackBytes(self, signalType, packedBytes, signal = ""):
        numBytes = struct.calcsize(f'={signalType}')
        if isinstance(signal, list):
            unpackedVariable = struct.unpack(f'={len(signal)}{signalType}', packedBytes[:len(signal)*numBytes])
            return list(unpackedVariable), packedBytes[len(unpackedVariable)*numBytes:]
        else:
            unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
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
    controller = Controller(args)
    controller.mainThread()

if __name__ == '__main__':
    main()