#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++ QUESTIONS+++++++++++++++++++++++++++++++++++++++ 
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

#Q1: Can you efine the suitable controller (Type:PD, data inputs) to control the pendulumn movement?
#Q2: Can design a PD controller in Python for me?
#Q3: How can I implement PD controller in EXUDYN model?
#Q4: Can you help me tunning the parameters of PD controller?
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

# Dimensions in meters 
a           = 0.1           # height/width of beam (m)
b           = a             # height/width of beam (m)
h           = 0.5* a       # Mesh size
L           = 1             # Length of beam (m)

# Material properties
rho         = 7800          # Density in kg/m^3
Emodulus    = 2.1e9        # Young's Modulus (Pa)
nu          = 0.3           # Poisson's Ratio
nModes      = 8             # Modes for dynamic simulations and dynamic analysis

import exudyn as exu
from exudyn.utilities import *  # includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics  # only import if it does not conflict 
from exudyn.FEM import *
import numpy as np
import time
import ngsolve as ngs
from netgen.meshing import *
from netgen.csg import *

SC = exu.SystemContainer()
mbs = SC.AddSystem()

useGraphics = True
fileName = 'testData/netgenBrick'  # for load/save of FEM data


meshCreated = True 
meshOrder = 1  # use order 2 for higher accuracy, but more unknowns
  
geo = CSGeometry()
block = OrthoBrick(Pnt(0, -a, -b), Pnt(L, a, b))
geo.Add(block)
mesh = ngs.Mesh(geo.GenerateMesh(maxh=h))
mesh.Curve(1)

if False:  # set this to true, if you want to visualize the mesh inside netgen/ngsolve
      import netgen.gui
      ngs.Draw(mesh)
      for i in range(100):
          netgen.Redraw()  # this makes the window interactive
          time.sleep(0.05)
          
fem = FEMinterface()
[bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho,
                                              youngsModulus=Emodulus,
                                              poissonsRatio=nu,
                                              meshOrder=meshOrder)
  
# Was able to formulate the joint (boundary cconditions)?
pLeft = [0, -a, -b]
pRight = [L, -a, -b]
nTip = fem.GetNodeAtPoint(pRight) #for sensor
nodesLeftPlane = fem.GetNodesInPlane(pLeft, [-1, 0, 0])
weightsLeftPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesLeftPlane)
nodesRightPlane = fem.GetNodesInPlane(pRight, [-1, 0, 0])
weightsRightPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesRightPlane)


boundaryList = [nodesLeftPlane]

print("nNodes=", fem.NumberOfNodes())
print("compute HCB modes... ")
start_time = time.time()
fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList,
                                    nEigenModes=nModes,
                                    useSparseSolver=True,
                                    computationMode=HCBstaticModeSelection.RBE2)
#print("HCB modes needed", )
if True:
      mat = KirchhoffMaterial(Emodulus, nu, rho)
      varType = exu.OutputVariableType.StressLocal
      print("ComputePostProcessingModes ... (may take a while)")
      start_time = time.time()
      fem.ComputePostProcessingModesNGsolve(fes, material=mat,
                                            outputVariableType=varType)
      #print("   ... needed
      SC.visualizationSettings.contour.reduceRange = False
      SC.visualizationSettings.contour.outputVariable = varType
      SC.visualizationSettings.contour.outputVariableComponent = 0  # x-component 
else:
      SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.DisplacementLocal
      SC.visualizationSettings.contour.outputVariableComponent = 1


  #print("create CMS element ...")
  
#Was able to define the flexible body?
cms = ObjectFFRFreducedOrderInterface(fem)
objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0, 0, 0],
                                          initialVelocity=[0, 0, 0],
                                          initialAngularVelocity=[0, 0, 0],
                                          gravity=[0, -9.806, 0],
                                          color=[0.1, 0.9, 0.1, 1.])

nodeDrawSize = 0.0025  # for joint drawing
mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))
leftMidPoint = [0, 0, 0]
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=leftMidPoint))
mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'],
                                                meshNodeNumbers=np.array(nodesLeftPlane),
                                                weightingFactors=weightsLeftPlane))

  
# Was able to formulate the joint (Revolue joint z-axis)?
mbs.AddObject(GenericJoint(markerNumbers=[mGround, mLeft],
                             constrainedAxes=[1, 1, 1, 1, 1, 1 * 0],
                             visualization=VGenericJoint(axesRadius=0.1 * a, axesLength=0.1 * a)))


fileDir = 'solution/'
sensTipDispl = mbs.AddSensor(SensorSuperElement(bodyNumber=objFFRF['oFFRFreducedOrder'],
                                                  meshNodeNumber=nTip,
                                                  fileName=fileDir + 'nMidDisplacementCMS' + str(nModes) + 'Test.txt',
                                                  outputVariableType=exu.OutputVariableType.Displacement))

mRight = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'],
                                               meshNodeNumbers=np.array(nodesRightPlane),
                                               weightingFactors=weightsRightPlane))
from ControllerLibrary import Control
 
# Was able to suggest tuning parameters?
 
# Initialize the PID controller with appropriate gains
Controller = Control(Kp=0,
                     Ki=0,
                     Kd=0)
 
def ControlForce(mbs, t, itemIndex, u, v, k, d, f0):
         omega = 0.5
         phiDesired = pi*0.5*sin(omega*pi*t)
         dt = 1e-3
         P = Controller.P(phiDesired, u, dt)
         phiDesired_t = pi*0.5*omega*cos(omega*pi*t)

         D = Controller.D(phiDesired_t, v, dt)
         Force = P + D
 
         #torque = Control.PID(phiDesired, phi0, dt)
 
         return Force 
     
def ControlTorque(mbs, t, itemIndex, theta, dtheta, k, d, offset):
           omega = 0.005
           phiDesired = sin(omega*t*pi)
           dt = 1e-3
           P = Controller.P(phiDesired, theta, dt)
           phiDesired_t = pi*omega*cos(omega*t)

           D = Controller.D(phiDesired_t, dtheta, dt)
           Torque = P + D
           
           if t>1.5:
               U =0 
           elif 1.5<t<3:
               U = 1 
           elif 3<t<4.5:
               U = -1 
           else:
               U = 0
               
               
           return Torque

# Force         = mbs.CreateSpringDamper(bodyOrNodeList=[objFFRF, mGround],stiffness=0,damping=0,force=0,
#                                      springForceUserFunction=ControlForce)
 
Torque         = mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mGround, mRight],
                                                           stiffness=0,damping=0,offset=0,
                                                           springTorqueUserFunction=ControlTorque))
 
mbs.Assemble()
simulationSettings = exu.SimulationSettings()   
 
#add dependencies of load user functions on nodal coordinates:
#ltgN0 = mbs.systemData.GetNodeLTGODE2(n0)
#both loads depend on both bodies; this information needs to be added in order that
#  the solver knows dependencies when computing Jacobians
#mbs.systemData.AddODE2LoadDependencies(lTorque0, list(ltgN0))
 
simulationSettings.timeIntegration.numberOfSteps  = 5000
simulationSettings.timeIntegration.endTime        = 5
simulationSettings.timeIntegration.newton.numericalDifferentiation.doSystemWideDifferentiation = True
simulationSettings.timeIntegration.computeLoadsJacobian = 2
# simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayStatistics = True
simulationSettings.displayComputationTime = True
mbs.SolveDynamic(simulationSettings=simulationSettings)          
mbs.SolutionViewer()
  