#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++ QUESTIONS+++++++++++++++++++++++++++++++++++++++ 
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

#Q1:(Interactive)Can you create a geometry of a rectangular beam with dimensions(?) 
#Q1:(Interactive)Can you create a geometry of a rectangular beam with dimensions(?) 
#                       with NgSolve and Exudyn?
#Q2: Can you mesh this geometry using Ngsolve and create FEM Interface?
#Q3: Can you help me in defining boundary conditions?
#Q4: Compute HurtyCraig BamptonModes using BC?
#Q5: How to create a flexible body using Ngsolve mesh in Exudyn (FFRF)?
#Q6: Can you create a revolute joint between the flexible body and ground?
#Q7: Solve this example using Simulation settings and show animation?
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


# add time integration scheme:
mbs.Assemble()
simulationSettings = exu.SimulationSettings()

SC.visualizationSettings.nodes.defaultSize = nodeDrawSize
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.connectors.defaultSize = 2 * nodeDrawSize
SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.sensors.show = True
SC.visualizationSettings.sensors.defaultSize = 0.01
SC.visualizationSettings.markers.show = False
SC.visualizationSettings.loads.drawSimplified = False

h = 1e-3
tEnd = 4

simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.solutionSettings.sensorsWritePeriod = h
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
simulationSettings.displayComputationTime = True

#Was able to run dynamic simulation ( running)?
mbs.SolveDynamic(simulationSettings=simulationSettings)

uTip = mbs.GetSensorValues(sensTipDispl)[1]
print("nModes=", nModes, ", tip displacement=", uTip)
 
 
#Was able to create animation?
mbs.SolutionViewer()    
  