

# DT Level 4: Relevant questions. 
    #Was able to define the suitable controller (Type:PID, data inputs)?
    #Was able to design PID controller?
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Details:  It includes PD controller on spring-damper, single pendulum, double 
            # pendulum and lift boom.
#
# Author:   Qasim Khadim
# Contact: qasim.khadim@outlook.com,qkhadim22 (Github)
# Date:     2025-02-02
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.FEM import *
import matplotlib.pyplot as plt
from exudyn.plot import PlotSensor, listMarkerStyles

import numpy as np
import time
import ngsolve as ngs
from netgen.meshing import *
from netgen.csg import *

from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict


from netgen.NgOCC import *

#import numpy as np
from math import sin, cos, sqrt,pi

import scipy.io
import os
import numpy as np

import math as mt
from math import sin, cos, sqrt, pi, tanh
import time

import os, sys

fileName1       = 'testData/Beam.stl'

class Control:
    
    def __init__(self, Kp=0, Ki=0, Kd=0, nStepsTotal=100, endTime=0.5):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.prev_error = 0  # Previous error value
        self.integral = 0  # Integral of error
        
        self.nStepsTotal = nStepsTotal
        self.endTime = endTime
        self.timeVecOut = np.arange(1,self.nStepsTotal+1)/self.nStepsTotal*self.endTime

#+++++++++++++++++++++P-Controller++++++++++++++++++++++++++++++++++++        
    def P(self, setpoint, current_value, dt):
        # Compute the error
        error = setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Update previous error
        self.prev_error = error
        
        return P
    
    #+++++++++++++++++++++PI-Controller++++++++++++++++++++++++++++++++++++        
    def D(self, setpoint, current_value, dt):
           # Compute the error
           error = setpoint - current_value
           
           
           # Derivative term
           D = self.Kd * (error - self.prev_error)
           
           # Update previous error
           self.prev_error = error
          
           return D
        

#+++++++++++++++++++++PI-Controller++++++++++++++++++++++++++++++++++++        
    def PI(self, setpoint, current_value, dt):
        # Compute the error
        error = setpoint - current_value
       
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        
        I = self.Ki * self.integral
                
        # Update previous error
        self.prev_error = error
        
        return  P + I

#+++++++++++++++++++++PID-Controller++++++++++++++++++++++++++++++++++++        
    def PID(self, setpoint, current_value, 
            dt):
        # Compute the error
        error = setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        
        I = self.Ki * self.integral
        
        # Derivative term
        D = self.Kd * (error - self.prev_error) / dt
        
        # Update previous error
        self.prev_error = error
        
        return P + I + D

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def CreateModel(self):
        self.SC  = exu.SystemContainer()
        self.mbs = self.SC.AddSystem()
    
    def GetOutputXAxisVector(self):
        return self.timeVecOut
    #get number of simulation steps
    def GetNSimulationSteps(self):
        return self.nStepsTotal # x finer simulation than output

    
    #initialState contains position and velocity states as list of two np.arrays 
    def Compute(self, SensorsData, verboseMode = 0, solutionViewer = False):
        self.CreateModel()
        # print('compute model')
        self.verboseMode = verboseMode

        

        self.inputTimeU  = np.zeros((self.nStepsTotal,2))
        self.inputTimeS  = np.zeros((self.nStepsTotal,2))
        self.inputTimedS = np.zeros((self.nStepsTotal,2))
        
        self.inputTimeU[:,0] = SensorsData.item()['t']
        self.inputTimeU[:,1] = SensorsData.item()['U']
        
        self.inputTimeS[:,0] = SensorsData.item()['t']
        self.inputTimeS[:,1] = SensorsData.item()['s']
        
        self.inputTimedS[:,0] = SensorsData.item()['t']
        self.inputTimedS[:,1] = SensorsData.item()['ds']

        self.mbs.variables['inputTimeU'] = self.inputTimeU
        self.mbs.variables['inputS'] = self.inputTimeS
        self.mbs.variables['inputdS'] = self.inputTimedS

        self.Simulation()

        if solutionViewer:
            self.mbs.SolutionViewer()
            
        # Inputs and outputs from the simulation.
        DS = self.dictSensors
        
        
        #++++++++++++++++++++++++++
        #Input data
        inputDict['t'] =  self.timeVecOut
        inputDict['U'] =  inputDict['U']

        
        #Outputdata
        # Joint 1
        
        # Actuator data
        outputData['s']  = self.mbs.GetSensorStoredData(DS['sDistance'])[0:self.nStepsTotal,1]
        outputData['ds'] = self.mbs.GetSensorStoredData(DS['sVelocity'])[0:self.nStepsTotal,1]
        
            
        return [inputDict, outputData] 
    
    
    #main function to create hydraulics arm
    def Simulation(self):
                
        self.x           = 0.2
        self.dotx        = 0
        self.dictSensors = {} 
        a           = 0.1           # height/width of beam (m)
        b           = a             # height/width of beam (m)
        h           = 0.5* a       # Mesh size
        L           = 1             # Length of beam (m)


        background = [graphics.CheckerBoard(point=[L,0,-2*b],size=5)]
        # background += [graphics.Cylinder(pAxis=[0,-0.25*L-0.5*b,-0.5*b], vAxis= [0,0,1.*b], radius = 0.25*b, 
        #                                      color= graphics.color.grey, addEdges=True, nTiles=32)]

        oGround=self.mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= background)))
        massRigid = 12*10
        inertiaRigid = massRigid/12*(L)**2
        g = 9.81    # gravity
        

        colCyl = graphics.color.orange
        colArm = graphics.color.green
        graphicsList = [graphics.Brick(size= [L,0.75*b,1.4*b], color= colArm, addEdges=True)]
        
        graphicsList += [graphics.Cylinder(pAxis=[-0.5*L,0,-0.75*b], vAxis= [0,0,1.5*b], radius = 0.55*b, 
                                     color= colArm, addEdges=True, nTiles=32)]

        graphicsList += [graphics.Cylinder(pAxis=[-0.5*L,0,-0.8*b], vAxis= [0,0,1.6*b], radius = 0.25*b, 
                                     color= graphics.color.grey, addEdges=True, nTiles=32)]

        #bolt
        graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b,-0.7*b], vAxis= [0,0,1.4*b], radius = 0.15*b, 
                                     color= graphics.color.grey, addEdges=True, nTiles=32)]

        graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b,-0.6*b], vAxis= [0,0,0.25*b], radius = 0.3*b, 
                                     color= colArm, addEdges=True, nTiles=32)]
        graphicsList += [graphics.Cylinder(pAxis=[-0.25*L,-0.5*b, 0.6*b], vAxis= [0,0,-0.25*b], radius = 0.3*b, 
                                     color= colArm, addEdges=True, nTiles=32)] 

        nRigid       = self.mbs.AddNode(Rigid2D(referenceCoordinates=[0.5*L,0,0], initialVelocities=[0,0,0]));
        oRigid       = self.mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,
                                   visualization=VObjectRigidBody2D(graphicsData= graphicsList)))

        mR1          = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5*L,0.,0.])) #support point
        mCOM1        = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) 
        mR1end       = self. mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0.5*L,0.,0.])) #end point

        #add joint
        mG0         = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
        self.mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1]))

        self.mbs.AddLoad(Force(markerNumber = mCOM1, loadVector = [0, -massRigid*g, 0]))

        mGH         = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,-0.25*L-0.5*b,0.]))
        mRH         = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid,  localPosition=[-0.25*L,-0.5*b,0.]))
            
        from exudyn.signalProcessing import GetInterpolatedSignalValue
        Controller          = Control(self.Kp,self.Ki,self.Kd)
        
        def ControlForce(mbs, t, itemIndex, u, v, k, d, f0):
           U  = GetInterpolatedSignalValue(t, mbs.variables['inputTimeU'], timeArray= [], dataArrayIndex= 1, 
                                               timeArrayIndex= 0, rangeWarning= False)
           
           s  = GetInterpolatedSignalValue(t, mbs.variables['inputS'], timeArray= [], dataArrayIndex= 1, 
                                               timeArrayIndex= 0, rangeWarning= False)
           
           ds = GetInterpolatedSignalValue(t, mbs.variables['inputdS'], timeArray= [], dataArrayIndex= 1, 
                                               timeArrayIndex= 0, rangeWarning= False)
           
           dt = 5e-3
           P = Controller.P(s, u, dt)
           D = Controller.D(ds, v, dt)
           Force = P + D+U
  
           return -Force
       
        ControlF = self.mbs.AddObject(SpringDamper(markerNumbers=[mGH, mRH],
                                  springForceUserFunction=ControlForce,visualization=VSpringDamper(show=False)
                                  ))

        self.mbs.Assemble()
        self.simulationSettings = exu.SimulationSettings()   
        self.simulationSettings.solutionSettings.sensorsWritePeriod = self.endTime / (self.nStepsTotal)
        self.simulationSettings.timeIntegration.newton.numericalDifferentiation.doSystemWideDifferentiation = True
        self.simulationSettings.timeIntegration.computeLoadsJacobian = 2


        
        self.simulationSettings.timeIntegration.numberOfSteps            = self.GetNSimulationSteps()
        self.simulationSettings.timeIntegration.endTime                  = self.endTime
        self.simulationSettings.timeIntegration.verboseModeFile          = 0
        self.simulationSettings.timeIntegration.verboseMode              = self.verboseMode
        self.simulationSettings.displayStatistics                        = True
        self.simulationSettings.displayComputationTime                   = True
        self.simulationSettings.linearSolverSettings.ignoreSingularJacobian=True
        
        #add dependencies of load user functions on nodal coordinates:
        #ltgN0 = mbs.systemData.GetNodeLTGODE2(n0)
        #both loads depend on both bodies; this information needs to be added in order that
        #  the solver knows dependencies when computing Jacobians
        #mbs.systemData.AddODE2LoadDependencies(lTorque0, list(ltgN0))
         
        self.mbs.SolveDynamic(simulationSettings=self.simulationSettings)          
        self.mbs.SolutionViewer()
        

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

