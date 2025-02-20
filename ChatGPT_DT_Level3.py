# -*- coding: utf-8 -*-
"""
ChatGPT 4
"""

ChatGPT4    = False
Copilot     = True 
ChatGPTMini = False


if ChatGPT4:
    import exudyn as exu
    from exudyn.utilities import *
    from exudyn.FEM import *
    import ngsolve as ngs
    from netgen.csg import *
    import numpy as np
    import time

    # Define geometry and mesh
    L, w, h = 1, 0.1, 0.1
    density, E, nu = 7800, 2.1e9, 0.3
    nModes = 8
    
    # Create geometry and mesh
    geo = CSGeometry()
    block = OrthoBrick(Pnt(0, -w/2, -h/2), Pnt(L, w/2, h/2))
    geo.Add(block)
    mesh = ngs.Mesh(geo.GenerateMesh(maxh=0.1))
    mesh.Curve(1)

    # Ngsolve FEM setup
    fem = FEMinterface()
    [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=density, youngsModulus=E, poissonsRatio=nu, meshOrder=1)

    # Exudyn system
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    nodesLeftPlane = fem.GetNodesInPlane([0, 0, 0], [-1, 0, 0])
    weightsLeftPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesLeftPlane)

    # HCB modes
    fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=[nodesLeftPlane], nEigenModes=nModes, useSparseSolver=True)

    # Compute stress post-processing modes
    mat = KirchhoffMaterial(E, nu, density)
    fem.ComputePostProcessingModesNGsolve(fes, material=mat, outputVariableType=exu.OutputVariableType.StressLocal)

    # FFRF object
    cms = ObjectFFRFreducedOrderInterface(fem)
    objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0, 0, 0], initialVelocity=[0, 0, 0], initialAngularVelocity=[0, 0, 0], gravity=[0, -9.81, 0], color=[0.1, 0.9, 0.1, 1])

    # Ground and joint setup
    oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0]))
    mLeft = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=objFFRF['oFFRFreducedOrder'], meshNodeNumbers=np.array(nodesLeftPlane), weightingFactors=weightsLeftPlane))
    mbs.AddObject(GenericJoint(markerNumbers=[mGround, mLeft], constrainedAxes=[1, 1, 1, 1, 1, 0], visualization=VGenericJoint(axesRadius=0.1 * w, axesLength=0.1 * L)))
    
    # Simulation setup
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = int(5 / 1e-3)
    simulationSettings.timeIntegration.endTime = 5.0
    simulationSettings.solutionSettings.solutionWritePeriod = 0.01
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
    
    # Visualization settings
    SC.visualizationSettings.nodes.defaultSize = 0.0025
    SC.visualizationSettings.connectors.defaultSize = 0.005
    SC.visualizationSettings.sensors.defaultSize = 0.01
    SC.visualizationSettings.nodes.show = False
    SC.visualizationSettings.markers.show = False
    SC.visualizationSettings.loads.drawSimplified = False
    SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.StressLocal
    SC.visualizationSettings.contour.outputVariableComponent = 0  # x-component of stress
    SC.visualizationSettings.contour.reduceRange = False
    
    # Run simulation
    exu.StartRenderer()
    mbs.SolveDynamic(simulationSettings)
    exu.StopRenderer()
    mbs.WaitForUserToContinue()

elif Copilot:
    
    a = 0.1
    b = a
    h = 0.5 * a
    L = 1

    rho = 7800
    Emodulus = 2.1e9
    nu = 0.3
    nModes = 8

    import exudyn as exu
    from exudyn.utilities import *
    import exudyn.graphics as graphics
    from exudyn.FEM import *
    import numpy as np
    import time
    import ngsolve as ngs
    from netgen.meshing import *
    from netgen.csg import *

    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    useGraphics = True
    fileName = 'testData/netgenBrick'

    meshCreated = True 
    meshOrder = 1
  
    geo = CSGeometry()
    block = OrthoBrick(Pnt(0, -a, -b), Pnt(L, a, b))
    geo.Add(block)
    mesh = ngs.Mesh(geo.GenerateMesh(maxh=h))
    mesh.Curve(1)
    
    fem = FEMinterface()
    [bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density=rho,
                                            youngsModulus=Emodulus,
                                            poissonsRatio=nu,
                                            meshOrder=meshOrder)
  
    pLeft = [0, -a, -b]
    pRight = [L, -a, -b]
    nTip = fem.GetNodeAtPoint(pRight)
    nodesLeftPlane = fem.GetNodesInPlane(pLeft, [-1, 0, 0])
    weightsLeftPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesLeftPlane)
    nodesRightPlane = fem.GetNodesInPlane(pRight, [-1, 0, 0])
    weightsRightPlane = fem.GetNodeWeightsFromSurfaceAreas(nodesRightPlane)
    
    boundaryList = [nodesLeftPlane]

    fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList,
                                  nEigenModes=nModes,
                                  useSparseSolver=True,
                                  computationMode=HCBstaticModeSelection.RBE2)

    mat = KirchhoffMaterial(Emodulus, nu, rho)
    varType = exu.OutputVariableType.StressLocal
    fem.ComputePostProcessingModesNGsolve(fes, material=mat,
                                          outputVariableType=varType)
    SC.visualizationSettings.contour.reduceRange = False
    SC.visualizationSettings.contour.outputVariable = varType
    SC.visualizationSettings.contour.outputVariableComponent = 0

    cms = ObjectFFRFreducedOrderInterface(fem)
    objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0, 0, 0],
                                        initialVelocity=[0, 0, 0],
                                        initialAngularVelocity=[0, 0, 0],
                                        gravity=[0, -9.806, 0],
                                        color=[0.1, 0.9, 0.1, 1.])

    nodeDrawSize = 0.0025
    mRB = mbs.AddMarker(MarkerNodeRigid(nodeNumber=objFFRF['nRigidBody']))
    oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))
    leftMidPoint = [0, 0, 0]
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
    
    mbs.AddObject(ObjectJointRevoluteZ(markerNumbers=[mGround, mRB]))

    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.endTime = 5
    simulationSettings.timeIntegration.numberOfSteps = 10000
    
    exu.SolveDynamic(mbs, simulationSettings)

    SC.visualizationSettings.nodes.defaultSize = 0.01
    SC.visualizationSettings.openGL.showFaceEdges = True
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

