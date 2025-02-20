#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

#Q1: Can you efine the suitable controller (Type:PD, data inputs) to control the pendulumn movement?
#Q2: Can design a PD controller in Python for me?
#Q3: How can I implement PD controller in EXUDYN model?
#Q4: Can you help me tunning the parameters of PD controller?
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
import exudyn as exu

from exudyn.utilities import *
import sys
import numpy as np
import time
   
#To create sensordata, choose RealSystem=True
RealSystem=False

Controller= True
# Simulation settings
timeStep    = 5e-3              #Simulation time step
T           = 5                #Time period
ns          = int(T/timeStep)        #Number of steps
angleInit   = -5
dataPath    = 'solution/Sensordata' + str(T) + '-' + 's' + str(ns) + 'Steps'


if RealSystem:

   from Models import Hydraulics
   dataPath  = 'solution/Sensordata' + str(T) + '-' + 's' + str(ns) + 'Steps'
   model     = Hydraulics(nStepsTotal=ns, endTime=T, verboseMode=1)
   model.CreateModel()
   inputVec =model.CreateInputVector( ns,  angleInit)
   data = model.ComputeModel(inputVec, solutionViewer = True)
   
   data_array = np.array(data, dtype=object)
   np.save(dataPath, data_array)
   
if Controller:
    from ControllerLibrary import Control
    
    Controller = Control(Kp=2e5,
                         Ki=0,
                         Kd=1e7,nStepsTotal=ns, endTime=T)
    
    SensorsData = np.load('solution/Sensordata5-s1000Steps.npy', allow_pickle=True)
    
    Control = Controller.Compute(SensorsData, solutionViewer = True)

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    



   