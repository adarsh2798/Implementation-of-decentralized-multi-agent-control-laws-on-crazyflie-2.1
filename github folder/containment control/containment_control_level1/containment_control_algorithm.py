import numpy as np


def find_GLT_roots(neighbor_pos):
    z1 = complex(neighbor_pos[0][0],neighbor_pos[0][1] )
    z2 = complex(neighbor_pos[1][0],neighbor_pos[1][1])
    z3 = complex(neighbor_pos[2][0],neighbor_pos[2][1])

    coefficients = [3, -2* (z1 + z2 + z3), z1 * z2 + z1 * z3 + z2 * z3]

    # Find the roots of the cubic polynomial
    roots = np.roots(coefficients)

    ans=[[roots.real[0],roots.imag[0]],[roots.real[1],roots.imag[1]]]
    return ans