from sympy import Matrix

def symbolic_inverse_matrix(matrix):
    try:
        inverse = matrix.inv()
        return inverse
    except ValueError:
        return None  # Return None if the matrix is not invertible

# Example usage:
# Define a symbolic matrix
A = Matrix([[1, 2, 3], [0, 1, 4], [5, 6, 0]])

# Get the inverse matrix
inverse_A = symbolic_inverse_matrix(A)

# Print the result
if inverse_A:
    print("Inverse of A:")
    print(inverse_A)
else:
    print("Matrix A is not invertible.")
