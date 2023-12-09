import sympy as sp

# Define symbolic variables
theta1, theta2, theta3 = sp.symbols('theta1 theta2 theta3')

# Lengths of links
l1, l2, l3 = 1, 1, 1  # Replace with actual lengths

# Forward kinematics equations
x = l1 * sp.cos(theta1) + l2 * sp.cos(theta1 + theta2) + l3 * sp.cos(theta1 + theta2 + theta3)
y = l1 * sp.sin(theta1) + l2 * sp.sin(theta1 + theta2) + l3 * sp.sin(theta1 + theta2 + theta3)

# Given end-effector position (replace with desired values)
x_desired = 2
y_desired = 2

# Solve inverse kinematics
# Substitute the desired position into the forward kinematics equations
inverse_solution = sp.solve((x - x_desired, y - y_desired), (theta1, theta2, theta3))

# Print the solutions
print("Inverse Kinematics Solutions:")
for solution in inverse_solution:
    print(f"Theta1: {solution[theta1]}, Theta2: {solution[theta2]}, Theta3: {solution[theta3]}")
