#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++ QUESTIONS+++++++++++++++++++++++++++++++++++++++ 
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

#Q1:(Interactive)Can you create a geometry of a rectangular beam with dimensions(?) 
#Q2: Can you mesh this geometry using elements and size?
#Q3: Can you help me in defining boundary conditions fixed joint at one end of beam?
#Q4: Can you help me in defining boundary conditions of force at other beam end?
#Q5: Can you create a 3D geometry and save it?
#Q6: Perfom static analysis?
#Q7: Compute deformation and stress at the beam end?
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 



import numpy as np


# Dimensions in meters 
a           = 0.1           # height/width of beam (m)
b           = a             # height/width of beam (m)
h           = 0.5* a       # Mesh size
L           = 1             # Length of beam (m)

# Material properties
rho         = 7800          # Density in kg/m^3
Emodulus    = 2.1e9        # Young's Modulus (Pa)
nu          = 0.3           # Poisson's Ratio

Static      = True
Geometry    = False
PlotMesh    = False

from ansys.mapdl.core import launch_mapdl

# Launch MAPDL
mapdl = launch_mapdl()
mapdl.clear()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++ DT Level 1: 3D Model+++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

# Pre-processing setup
mapdl.prep7()
mapdl.units("SI")
# Create geometry - block as cantilever beam

mapdl.blc4(width=L, height=a, depth=b)

if Geometry:
    
    filePath ='solution\3D Model'
    #mapdl.vplot()
    mapdl.vplot(show_lines=True)
    #Can we save the file?        

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++ DT Level 2: Static analysis++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

# Set material properties
mapdl.mp('EX', 1, Emodulus)  # 3 Young's modulus
mapdl.mp('NUXY', 1, nu)  # Poisson's ratio
mapdl.mp('DENS', 1, rho)  # Density
#mapdl.acel(0, gravity, 0)


#MESH: Mesh the geometry using SOLID185 element
mapdl.et(1, 'SOLID185')  # Element type SOLID185
mapdl.esize(h)  # Element size
mapdl.vmesh('ALL')

# # Post-processing: Element Plot
# mapdl.post1()

if PlotMesh:

    mapdl.eplot(title="Element Plot")
 
if Static:
    #BOUNDARY CONDITIONS: Applying contraints

    mapdl.nsel('S', 'LOC', 'X', 0)  # Select nodes on the fixed end
    mapdl.nsel('S', 'LOC', 'Y',0)  # Select nodes on the fixed end
    mapdl.nsel('S', 'LOC', 'Z', -a/2)  # Select nodes on the fixed end
    
    mapdl.d('ALL', 'ALL')  # Fix all degrees of freedom
    mapdl.allsel()

    #BOUNDARY CONDITIONS: Apply a force on the free end
    mapdl.nsel('S', 'LOC', 'X', L)  # Select nodes at the free end
    #mapdl.nsel('S', 'LOC', 'Y', 0.0)  # Refine selection to Y = 0.0
    #mapdl.nsel('S', 'LOC', 'Z', 0.0)  # Refine selection to Z = 0.0

    mapdl.f('ALL', 'FY', -1)  # Apply force in Y-direction
    mapdl.allsel()
    
    
    #Static analysis
    mapdl.run('/SOLU')
    mapdl.antype('STATIC')
    mapdl.solve()
    mapdl.finish()

    mapdl.result.plot_nodal_displacement(0)
 
    # Post-processing: Element Plot
    mapdl.post1()
    mapdl.eplot(title="Element Plot")
 
    # Deformation Plot
    mapdl.set(1, 1)  # Set the first load step and substep
    mapdl.pldisp(title="Deformation Plot")
    
    result = mapdl.result
    print(result)
    
    result.plot_principal_nodal_stress(
        0,
    "SEQV",
    cpos="xy",
    background="w",
    text_color="k",
    add_text=True,
    show_edges=True,
    )
    nnum, stress = result.principal_nodal_stress(0)
 
    # Von Mises stress is the last column in the stress results
    von_mises = stress[:, -1]
 
    # Bending Stress Plot (SX)
    mapdl.set(1, 1)
    mapdl.plnsol('S', 'X', title="Bending Stress Plot (SX)")
 
    # Strain Plot
    mapdl.set(1, 1)
    mapdl.plnsol('EPEL', title="Strain Plot")
 
    # Close MAPDL session
mapdl.exit()