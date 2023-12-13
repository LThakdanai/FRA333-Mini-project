q1 = 30
q2 = 60
q3 = 0
# Link 1
vm1 = 2.492  # kg
vcom1x = -90.56 / 1000  # m
vcom1z = 114.74 / 1000

vIxx1 = 14354888.77 * 1e-9  # 1e9 = 1/1000^3 conversion from g mm^2 to kg m^2
vIyy1 = 29370510.38 * 1e-9
vIzz1 = 17526249.63 * 1e-9
vIxz1 = 11748431.07 * 1e-9

# Link 2
vm2 = 1.624  # kg
vcom2x = -6.31 / 1000  # m
vcom2z = -73.38 / 1000

vIxx2 = 8955973.99 * 1e-9
vIyy2 = 8508639.70 * 1e-9
vIzz2 = 1966741.10 * 1e-9
vIxz2 = -687364.95 * 1e-9

# Link 3
vm3 = 0.756  # kg
vcom3z = 31 / 1000

vIxx3 = 1260000.00 * 1e-9
vIyy3 = 655200.00 * 1e-9
vIzz3 = 1864800.00 * 1e-9

def q1 q2 q3:
    DH = sp.Matrix([
    [q1 + sp.pi / 2, 0, 0, sp.pi / 2],
    [q2 - sp.pi / 2, 0, 0, -sp.pi / 2],
    [q3, 0, 0, 0]
])

DH = sp.Matrix([
    [q1 + sp.pi / 2, 0, 0, sp.pi / 2],
    [q2 - sp.pi / 2, 0, 0, -sp.pi / 2],
    [q3, 0, 0, 0]
])

A01 = sp.Matrix([
    [sp.cos(DH[0, 0]), -sp.sin(DH[0, 0]) * sp.cos(DH[0, 3]), sp.sin(DH[0, 0]) * sp.sin(DH[0, 3]), 0],
    [sp.sin(DH[0, 0]), sp.cos(DH[0, 0]) * sp.cos(DH[0, 3]), -sp.cos(DH[0, 0]) * sp.sin(DH[0, 3]), 0],
    [0, sp.sin(DH[0, 3]), sp.cos(DH[0, 3]), 0],
    [0, 0, 0, 1]
])

A12 = sp.Matrix([
    [sp.cos(DH[1, 0]), -sp.sin(DH[1, 0]) * sp.cos(DH[1, 3]), sp.sin(DH[1, 0]) * sp.sin(DH[1, 3]), 0],
    [sp.sin(DH[1, 0]), sp.cos(DH[1, 0]) * sp.cos(DH[1, 3]), -sp.cos(DH[1, 0]) * sp.sin(DH[1, 3]), 0],
    [0, sp.sin(DH[1, 3]), sp.cos(DH[1, 3]), 0],
    [0, 0, 0, 1]
])

A23 = sp.Matrix([
    [sp.cos(DH[2, 0]), -sp.sin(DH[2, 0]) * sp.cos(DH[2, 3]), sp.sin(DH[2, 0]) * sp.sin(DH[2, 3]), 0],
    [sp.sin(DH[2, 0]), sp.cos(DH[2, 0]) * sp.cos(DH[2, 3]), -sp.cos(DH[2, 0]) * sp.sin(DH[2, 3]), 0],
    [0, sp.sin(DH[2, 3]), sp.cos(DH[2, 3]), 0],
    [0, 0, 0, 1]
])

T03 = A01 * A12 * A23

R01 = A01[0:3, 0:3]
R12 = A12[0:3, 0:3]
R23 = A23[0:3, 0:3]


JP = sp.zeros(3, 3)

z0 = sp.Matrix([0,0,1])
z1 = R01 * z0
z2 = R01 * R12 * z0

Jw3 = sp.Matrix.hstack(z0, z1, z2)

JO = Jw3

J = sp.Matrix.vstack(JP, JO) #48

Ixx1, Iyy1, Izz1, Ixz1 = sp.symbols('Ixx1 Iyy1 Izz1 Ixz1', real=True)

Il1 = sp.Matrix([
    [Ixx1, 0, -Ixz1],
    [0, Iyy1, 0],
    [-Ixz1, 0, Izz1]
])

Ixx2, Iyy2, Izz2, Ixz2 = sp.symbols('Ixx2 Iyy2 Izz2 Ixz2', real=True)

Il2 = sp.Matrix([
    [Ixx2, 0, -Ixz2],
    [0, Iyy2, 0],
    [-Ixz2, 0, Izz2]
])

Ixx3, Iyy3, Izz3 = sp.symbols('Ixx3 Iyy3 Izz3', real=True)

Il3 = sp.Matrix([
    [Ixx3, 0, 0],
    [0, Iyy3, 0],
    [0, 0, Izz3]
])

# Positions of the center of mass in the base frame 
# verified
pl1 = R01 * sp.Matrix([com1x, 0, com1z])
pl2 = R01 * R12 * sp.Matrix([com2x, 0, com2z])
pl3 = R01 * R12 * R23 * sp.Matrix([0, 0, com3z])



# Translational Jacobians
Jpl1 = sp.simplify(sp.Matrix([
    [z0.cross(pl1),
    sp.zeros(3,1),
    sp.zeros(3,1)]
])) # [(3x1) , (3x1) , (3x1)]

Jpl2 = sp.simplify(sp.Matrix([
    [z0.cross(pl1),
    z1.cross(pl2),
    sp.zeros(3,1)]
]))

Jpl3 = sp.simplify(sp.Matrix([
    [z0.cross(pl1),
    z1.cross(pl2),
    z2.cross(pl3)]
]))

# Rotational Jacobians
Jol1 = sp.Matrix([[
    z0,
    sp.zeros(3,1),
    sp.zeros(3,1)
]])

Jol2 = sp.Matrix([[
    z0,
    z1,
    sp.zeros(3,1)
]])

Jol3 = sp.Matrix([[
    z0,
    z1,
    z2
]])

#review
print('pl1', pl1)
print('pl2', pl2)
print('pl3', pl3)

print('Jpl1', Jpl1)
print('Jpl2', Jpl2)
print('Jpl3', Jpl3)

print('Jol1', Jol1)
print('Jol2', Jol2)
print('Jol3', Jol3)

print('--------------------Blank--------------------')

print ('z2 cross pl3', z2.cross(pl3))
print('New1',((m1 * (Jpl1.T * Jpl1)) + Jol1.T * R01 * Il1 * R01.T * Jol1))

R02 = R01 * R12
R03 = R02 * R23

# Inertia matrix calculations for individual components
B1 = (m1 * (Jpl1.T * Jpl1)) + Jol1.T * R01 * Il1 * R01.T * Jol1
B2 = (m2 * (Jpl2.T * Jpl2)) + Jol2.T * R02 * Il2 * R02.T * Jol2
B3 = (m3 * (Jpl3.T * Jpl3)) + Jol3.T * R03 * Il3 * R03.T * Jol3

# Total inertia matrix
B = sp.simplify(B1 + B2 + B3)


# Calculate the inverse of the inertia matrix
# invB = sp.Matrix([[                                                           

#reivew


# Define symbols
qdot1, qdot2, qdot3 = sp.symbols('qdot1 qdot2 qdot3', real=True)
assume_dict = {qdot1: True, qdot2: True, qdot3: True}


# Define B matrix elements (replace these with your actual values or symbols)
B_values = {B[i, j]: B[i, j].subs(assume_dict) for i in range(3) for j in range(3)}

# Function to compute cijk
def cijk(bij, bik, bjk, qi, qj, qk):
    return sp.simplify(0.5 * (sp.diff(bij, qk) + sp.diff(bik, qj) - sp.diff(bjk, qi)))


# Compute cijk values
c111 = cijk(B_values[B[0, 0]], B_values[B[0, 0]], B_values[B[0, 0]], q1, q1, q1)
c112 = cijk(B_values[B[0, 0]], B_values[B[0, 1]], B_values[B[1, 1]], q1, q1, q2)
c113 = cijk(B_values[B[0, 0]], B_values[B[0, 2]], B_values[B[2, 2]], q1, q1, q3)
c121 = c112
c122 = cijk(B_values[B[0, 1]], B_values[B[0, 1]], B_values[B[1, 1]], q1, q2, q2)
c123 = cijk(B_values[B[0, 1]], B_values[B[0, 2]], B_values[B[1, 2]], q1, q2, q3)
c131 = c113
c132 = c123
c133 = cijk(B_values[B[0, 2]], B_values[B[0, 2]], B_values[B[2, 2]], q1, q3, q3)

c211 = cijk(B_values[B[1, 0]], B_values[B[1, 0]], B_values[B[0, 0]], q2, q1, q1)
c212 = cijk(B_values[B[1, 0]], B_values[B[1, 1]], B_values[B[0, 1]], q2, q1, q2)
c213 = cijk(B_values[B[1, 0]], B_values[B[1, 2]], B_values[B[0, 2]], q2, q1, q3)
c221 = c212
c222 = cijk(B_values[B[1, 1]], B_values[B[1, 1]], B_values[B[1, 1]], q2, q2, q2)
c223 = cijk(B_values[B[1, 1]], B_values[B[1, 2]], B_values[B[1, 2]], q2, q2, q3)
c231 = c213
c232 = c223
c233 = cijk(B_values[B[1, 2]], B_values[B[1, 2]], B_values[B[2, 2]], q2, q3, q3)

c311 = cijk(B_values[B[2, 0]], B_values[B[2, 0]], B_values[B[0, 0]], q3, q1, q1)
c312 = cijk(B_values[B[2, 0]], B_values[B[2, 1]], B_values[B[0, 1]], q3, q1, q2)
c313 = cijk(B_values[B[2, 0]], B_values[B[2, 2]], B_values[B[0, 2]], q3, q1, q3)
c321 = c312
c322 = cijk(B_values[B[2, 1]], B_values[B[2, 1]], B_values[B[1, 1]], q3, q2, q2)
c323 = cijk(B_values[B[2, 1]], B_values[B[2, 2]], B_values[B[1, 2]], q3, q2, q3)
c331 = c313
c332 = c323
c333 = cijk(B_values[B[2, 2]], B_values[B[2, 2]], B_values[B[2, 2]], q3, q3, q3)

# Compute c values
c11 = c111 * qdot1 + c112 * qdot2 + c113 * qdot3
c12 = c121 * qdot1 + c122 * qdot2 + c123 * qdot3
c13 = c131 * qdot1 + c132 * qdot2 + c133 * qdot3
c21 = c211 * qdot1 + c212 * qdot2 + c213 * qdot3
c22 = c221 * qdot1 + c222 * qdot2 + c223 * qdot3
c23 = c231 * qdot1 + c232 * qdot2 + c233 * qdot3
c31 = c311 * qdot1 + c312 * qdot2 + c313 * qdot3
c32 = c321 * qdot1 + c322 * qdot2 + c323 * qdot3
c33 = c331 * qdot1 + c332 * qdot2 + c333 * qdot3

# Formulate C matrix
C = sp.Matrix([[c11, c12, c13], [c21, c22, c23], [c31, c32, c33]])
C_simplified = C.simplify()


#NOT APPROVE

g0 = sp.Matrix([0, 0, 9.8])

# Calculate g1, g2, g3
g1 = sp.simplify(-m1 * g0.dot(Jpl1[:, 0]) - m2 * g0.dot(Jpl2[:, 0]) - m3 * g0.dot(Jpl3[:, 0]))
g2 = sp.simplify(-m1 * g0.dot(Jpl1[:, 1]) - m2 * g0.dot(Jpl2[:, 1]) - m3 * g0.dot(Jpl3[:, 1]))
g3 = sp.simplify(-m1 * g0.dot(Jpl1[:, 2]) - m2 * g0.dot(Jpl2[:, 2]) - m3 * g0.dot(Jpl3[:, 2]))

G = sp.Matrix([g1, g2, g3])
#review
print('g1 =',g1)
print('g2 =',g2)
print('g3 =',g3)

print('G =', G)

# Define symbols
qdotdot1, qdotdot2, qdotdot3 = sp.symbols('qdotdot1 qdotdot2 qdotdot3')
Fv1, Fv2, Fv3 = sp.symbols('Fv1 Fv2 Fv3')
tau1, tau2, tau3 = sp.symbols('tau1 tau2 tau3')

# Define qdotdot, qdot, Fv, tau
qdotdot = sp.Matrix([qdotdot1, qdotdot2, qdotdot3])
qdot = sp.Matrix([qdot1, qdot2, qdot3])
Fv = sp.diag(Fv1, Fv2, Fv3)
tau = sp.Matrix([tau1, tau2, tau3])

# Compute TAU and QDD
TAU = B * qdotdot + C * qdot + Fv * qdot + G
QDD = invB * (tau - C * qdot - Fv * qdot - G)

print(Fv)
print(TAU)
print(QDD)

return tau1 tau2 tau3 
