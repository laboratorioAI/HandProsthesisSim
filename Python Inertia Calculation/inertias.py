import FreeCAD as App
import Part
import numpy as np

obj = App.ActiveDocument.getObject("GenericProximalPhalanx")  # Replace with your object name
shape = obj.Shape
solids = shape.Solids

if not solids:
    raise Exception("No solids found in the object.")

density = 1273.2756  # kg/m³

# Accumulators
total_mass = 0
weighted_com = App.Vector(0, 0, 0)

for solid in solids:
    volume_mm3 = solid.Volume
    mass = (density / 1e9) * volume_mm3
    com = solid.CenterOfMass
    weighted_com += com.multiply(mass)
    total_mass += mass

# Final center of mass
com_final = weighted_com.multiply(1 / total_mass)

# Now recalculate total inertia with parallel axis theorem
inertia_total = App.Matrix()

for solid in solids:
    volume_mm3 = solid.Volume
    mass = (density / 1e9) * volume_mm3
    com = solid.CenterOfMass
    inertia = solid.MatrixOfInertia  # Inertia at solid's COM

    # Displacement vector from total COM
    r = com.sub(com_final)
    dx, dy, dz = r.x, r.y, r.z

    # Offset inertia using parallel axis theorem
    offset = App.Matrix()
    offset.A11 = mass * (dy*dy + dz*dz)
    offset.A22 = mass * (dx*dx + dz*dz)
    offset.A33 = mass * (dx*dx + dy*dy)
    offset.A12 = offset.A21 = -mass * dx * dy
    offset.A13 = offset.A31 = -mass * dx * dz
    offset.A23 = offset.A32 = -mass * dy * dz

    # Total inertia for solid = local + offset
    corrected = App.Matrix()
    corrected.A11 = inertia.A11 + offset.A11
    corrected.A12 = inertia.A12 + offset.A12
    corrected.A13 = inertia.A13 + offset.A13
    corrected.A21 = inertia.A21 + offset.A21
    corrected.A22 = inertia.A22 + offset.A22
    corrected.A23 = inertia.A23 + offset.A23
    corrected.A31 = inertia.A31 + offset.A31
    corrected.A32 = inertia.A32 + offset.A32
    corrected.A33 = inertia.A33 + offset.A33

    # Accumulate
    inertia_total.A11 += corrected.A11
    inertia_total.A12 += corrected.A12
    inertia_total.A13 += corrected.A13
    inertia_total.A21 += corrected.A21
    inertia_total.A22 += corrected.A22
    inertia_total.A23 += corrected.A23
    inertia_total.A31 += corrected.A31
    inertia_total.A32 += corrected.A32
    inertia_total.A33 += corrected.A33

# Display basic properties
print(f"Total Mass (kg): {total_mass:.6f}")
print(f"Center of Mass (mm): X={com_final.x:.4f}, Y={com_final.y:.4f}, Z={com_final.z:.4f}")
print("Corrected Inertia Matrix (kg·mm²):")
print(inertia_total)

# Build inertia matrix in numpy
I_mat = np.array([
    [inertia_total.A11, inertia_total.A12, inertia_total.A13],
    [inertia_total.A21, inertia_total.A22, inertia_total.A23],
    [inertia_total.A31, inertia_total.A32, inertia_total.A33]
])

# Diagonalize
eigvals, eigvecs = np.linalg.eigh(I_mat)
sorted_indices = np.argsort(eigvals)
eigvals_sorted = eigvals[sorted_indices]
eigvecs_sorted = eigvecs[:, sorted_indices]

print("\nPrincipal Moments of Inertia (kg·mm²):")
for i, val in enumerate(eigvals_sorted):
    print(f"I_{i+1} = {val:.6f}")

print("\nPrincipal Axes (columns):")
print(eigvecs_sorted)
