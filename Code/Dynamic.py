import sympy as sp
import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Link 1
m1 = 2.492  # kg
com1x = -90.56 / 1000  # m
com1z = 114.74 / 1000

Ixx1 = 14354888.77 * 1e-9  # 1e9 = 1/1000^3 conversion from g mm^2 to kg m^2
Iyy1 = 29370510.38 * 1e-9
Izz1 = 17526249.63 * 1e-9
Ixz1 = 11748431.07 * 1e-9

# Link 2
m2 = 1.624  # kg
com2x = -6.31 / 1000  # m
com2z = -73.38 / 1000

Ixx2 = 8955973.99 * 1e-9
Iyy2 = 8508639.70 * 1e-9
Izz2 = 1966741.10 * 1e-9
Ixz2 = -687364.95 * 1e-9

# Link 3
m3 = 0.756  # kg
com3z = 31 / 1000

Ixx3 = 1260000.00 * 1e-9
Iyy3 = 655200.00 * 1e-9
Izz3 = 1864800.00 * 1e-9


TAU_numpy = None

def dynamic(q1,q2,q3):
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

    #review
    


    #Transalational Part of the jacobian in null

    JP = sp.zeros(3, 3)

    z0 = sp.Matrix([0,0,1])
    z1 = R01 * z0
    z2 = R01 * R12 * z0

    Jw3 = sp.Matrix.hstack(z0, z1, z2)

    JO = Jw3

    J = sp.Matrix.vstack(JP, JO) #48
    
    Il1 = sp.Matrix([
        [Ixx1, 0, -Ixz1],
        [0, Iyy1, 0],
        [-Ixz1, 0, Izz1]
    ])

    Il2 = sp.Matrix([
        [Ixx2, 0, -Ixz2],
        [0, Iyy2, 0],
        [-Ixz2, 0, Izz2]
    ])

    Il3 = sp.Matrix([
        [Ixx3, 0, 0],
        [0, Iyy3, 0],
        [0, 0, Izz3]
    ])

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

    #NOT APPROVE.... Can't use function Inverse 

    #Inertia matrix
    R02 = R01 * R12
    R03 = R02 * R23

    # Inertia matrix calculations for individual components
    B1 = (m1 * (Jpl1.T * Jpl1)) + Jol1.T * R01 * Il1 * R01.T * Jol1
    B2 = (m2 * (Jpl2.T * Jpl2)) + Jol2.T * R02 * Il2 * R02.T * Jol2
    B3 = (m3 * (Jpl3.T * Jpl3)) + Jol3.T * R03 * Il3 * R03.T * Jol3

    # Total inertia matrix
    B = sp.simplify(B1 + B2 + B3)


    # Calculate the inverse of the inertia matrix

     # Define symbols
    
    qdot1 = 0.1
    qdot2 = 0.1
    qdot3 = 0

    # Function to compute cijk
    def cijk(bij, bik, bjk, qi, qj, qk):
        return (0.5 * (sp.diff(bij, qk) + sp.diff(bik, qj) - sp.diff(bjk, qi)))
    
    # Define B matrix elements (replace these with your actual values or symbols)
    #for i in range(3):
    #    for j in range(3):
    #        print(f"B_value[{i},{j}]:", B[i, j])

    # Compute cijk values
    c111 = 0
    c112 = -0.5*Ixx2*sp.sin(2*q2) - 1.0*Ixx3*sp.sin(q2)*sp.cos(q2)*sp.cos(q3)**2 + 1.0*Ixz2*sp.cos(2*q2) - 1.0*Iyy3*sp.sin(q2)*sp.sin(q3)**2*sp.cos(q2) + 0.5*Izz2*sp.sin(2*q2) + 0.5*Izz3*sp.sin(2*q2)
    c113 = 1.0*(-Ixx3 + Iyy3)*sp.sin(q3)*sp.cos(q2)**2*sp.cos(q3)
    c121 = c112
    c122 = -0.25*Ixx3*sp.cos(q2 - 2*q3) + 0.25*Ixx3*sp.cos(q2 + 2*q3) + 0.25*Iyy3*sp.cos(q2 - 2*q3) - 0.25*Iyy3*sp.cos(q2 + 2*q3) - 1.0*com1z*com2x*m2*sp.sin(q2) - 1.0*com1z*com2x*m3*sp.sin(q2) - 1.0*com1z*com2z*m2*sp.cos(q2) - 1.0*com1z*com2z*m3*sp.cos(q2)
    c123 = (-1.0*Ixx3*sp.sin(q3)**2 + 0.5*Ixx3 + 1.0*Iyy3*sp.sin(q3)**2 - 0.5*Iyy3 + 0.5*Izz3)*sp.cos(q2)
    c131 = c113
    c132 = c123
    c133 = 0

    c211 = 0.5*Ixx2*sp.sin(2*q2) + 1.0*Ixx3*sp.sin(q2)*sp.cos(q2)*sp.cos(q3)**2 - 1.0*Ixz2*sp.cos(2*q2) + 1.0*Iyy3*sp.sin(q2)*sp.sin(q3)**2*sp.cos(q2) - 0.5*Izz2*sp.sin(2*q2) - 0.5*Izz3*sp.sin(2*q2)
    c212 = 0
    c213 = (-1.0*Ixx3*sp.sin(q3)**2 + 0.5*Ixx3 + 1.0*Iyy3*sp.sin(q3)**2 - 0.5*Iyy3 - 0.5*Izz3)*sp.cos(q2)
    c221 = c212
    c222 = 0
    c223 = 0.5*(Ixx3 - Iyy3)*sp.sin(2*q3)
    c231 = c213
    c232 = c223
    c233 = 0

    c311 = 1.0*(Ixx3 - Iyy3)*sp.sin(q3)*sp.cos(q2)**2*sp.cos(q3)
    c312 = (1.0*Ixx3*sp.sin(q3)**2 - 0.5*Ixx3 - 1.0*Iyy3*sp.sin(q3)**2 + 0.5*Iyy3 + 0.5*Izz3)*sp.cos(q2)
    c313 = 0
    c321 = c312
    c322 = 0.5*(-Ixx3 + Iyy3)*sp.sin(2*q3)
    c323 = 0
    c331 = c313
    c332 = c323
    c333 = 0
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

    # Output the results
   
    g0 = sp.Matrix([0, 0, 9.8])

    # Calculate g1, g2, g3
    g1 = sp.simplify(-m1 * g0.dot(Jpl1[:, 0]) - m2 * g0.dot(Jpl2[:, 0]) - m3 * g0.dot(Jpl3[:, 0]))
    g2 = sp.simplify(-m1 * g0.dot(Jpl1[:, 1]) - m2 * g0.dot(Jpl2[:, 1]) - m3 * g0.dot(Jpl3[:, 1]))
    g3 = sp.simplify(-m1 * g0.dot(Jpl1[:, 2]) - m2 * g0.dot(Jpl2[:, 2]) - m3 * g0.dot(Jpl3[:, 2]))

    G = sp.Matrix([g1, g2, g3])
    #review


    qdotdot1 = 0
    qdotdot2 = 0
    qdotdot3 = 0
    tau1, tau2, tau3 = sp.symbols('tau1 tau2 tau3')

    # Define qdotdot, qdot, Fv, tau
    qdotdot = sp.Matrix([qdotdot1, qdotdot2, qdotdot3])
    qdot = sp.Matrix([qdot1, qdot2, qdot3])
    Fv = sp.diag(1, 1, 1)
    TAU = sp.Matrix([tau1, tau2, tau3])

    TAU = B * qdotdot + C * qdot + Fv * qdot + G
    global TAU_numpy
    TAU_numpy = np.array(TAU).astype(np.float64)
    print("Tau",TAU_numpy)

    return TAU_numpy

