import FreeCAD as App
>>> import Part
>>> import numpy as np
>>> 
>>> # === CONFIGURATION ===
>>> obj = App.ActiveDocument.getObject("GenericMiddlePhalanx")
>>> 
>>> shape = obj.Shape
>>> solids = shape.Solids
>>> 
>>> if not solids:
...     raise Exception("No solids found in the object.")
... 
>>> # Density in kg/m³ (adjust if needed to match Inventor)
>>> density = 1273.2756
>>> 
>>> # === ACCUMULATORS ===
>>> total_mass = 0
>>> weighted_com = App.Vector(0, 0, 0)
>>> inertia_total = App.Matrix()
>>> 
>>> for solid in solids:
...     volume_mm3 = solid.Volume
...     mass = (density / 1e9) * volume_mm3  # Convert mm³ → m³
...     com = solid.CenterOfMass
...     inertia = solid.MatrixOfInertia  # In kg·mm², about global origin
... 
...     weighted_com += com.multiply(mass)
...     total_mass += mass
... 
...     inertia_total.A11 += inertia.A11
...     inertia_total.A12 += inertia.A12
...     inertia_total.A13 += inertia.A13
...     inertia_total.A21 += inertia.A21
...     inertia_total.A22 += inertia.A22
...     inertia_total.A23 += inertia.A23
...     inertia_total.A31 += inertia.A31
...     inertia_total.A32 += inertia.A32
...     inertia_total.A33 += inertia.A33
... 
>>> # === FINAL CENTER OF MASS ===
>>> com_final = weighted_com.multiply(1 / total_mass)
>>> 
>>> # === BUILD GLOBAL INERTIA MATRIX (about origin) ===
>>> I_global = np.array([
...     [inertia_total.A11, inertia_total.A12, inertia_total.A13],
...     [inertia_total.A21, inertia_total.A22, inertia_total.A23],
...     [inertia_total.A31, inertia_total.A32, inertia_total.A33]
... ])
>>> 
>>> # === PARALLEL AXIS THEOREM: SHIFT TO COM ===
>>> x, y, z = com_final.x, com_final.y, com_final.z
>>> D = np.array([
...     [y**2 + z**2, -x * y,      -x * z],
...     [-x * y,      x**2 + z**2, -y * z],
...     [-x * z,     -y * z,      x**2 + y**2]
... ])
>>> I_com = I_global - total_mass * D
>>> 
>>> # === DIAGONALIZE INERTIA TENSOR ABOUT COM ===
>>> eigvals_com, eigvecs_com = np.linalg.eigh(I_com)
>>> 
>>> # === CONVERT TO kg·cm² FOR COMPARISON WITH INVENTOR ===
>>> eigvals_cm2 = eigvals_com / 10000.0
>>> 
>>> # Optional: compare against Inventor values
>>> inventor_vals = np.array([12.4766, 11.9522, 13.4961])
>>> error = np.abs(eigvals_cm2 - inventor_vals) / inventor_vals * 100
>>> 
>>> # === OUTPUT ===
>>> print("\n=== Rigid Body Properties (COM Frame) ===")

=== Rigid Body Properties (COM Frame) ===
>>> print(f"Total Mass: {total_mass:.6f} kg")
Total Mass: 0.002637 kg
>>> print(f"Center of Mass (mm): X={com_final.x:.4f}, Y={com_final.y:.4f}, Z={com_final.z:.4f}")
Center of Mass (mm): X=12.0496, Y=-2.0097, Z=0.4008
>>> print("\nInertia Tensor (about Origin, Global Frame) [kg·mm²]:\n", I_global)

Inertia Tensor (about Origin, Global Frame) [kg·mm²]:
 [[ 95352.16764408  -7699.61769069  -3318.30336098]
 [ -7699.61769069  97301.83109583    484.08363276]
 [ -3318.30336098    484.08363276 100921.802128  ]]
>>> print("\nInertia Tensor (about COM) [kg·mm²]:\n", I_com)

Inertia Tensor (about COM) [kg·mm²]:
 [[ 95352.15657043  -7699.68154417  -3318.29062719]
 [ -7699.68154417  97301.44783392    484.0815089 ]
 [ -3318.29062719    484.0815089  100921.40863954]]
>>> print("\nPrincipal Moments (about COM) [kg·mm²]:", eigvals_com)

Principal Moments (about COM) [kg·mm²]: [ 88183.91345071  99810.873522   105580.22607118]
>>> print("Principal Axes (columns = unit vectors):\n", eigvecs_com)
Principal Axes (columns = unit vectors):
 [[ 0.75697307 -0.21222122 -0.61802421]
 [ 0.63005863  0.48781526  0.60420394]
 [ 0.17325674 -0.84675759  0.50297483]]
>>> print("\nPrincipal Moments (about COM) [kg·cm²]:", eigvals_cm2)

Principal Moments (about COM) [kg·cm²]: [ 8.81839135  9.98108735 10.55802261]
>>> print("\nInventor Comparison [kg·cm²]:", inventor_vals)

Inventor Comparison [kg·cm²]: [12.4766 11.9522 13.4961]
>>> print("Relative Error [%]:", error)
Relative Error [%]: [29.32055732 16.49163039 21.7698253 ]