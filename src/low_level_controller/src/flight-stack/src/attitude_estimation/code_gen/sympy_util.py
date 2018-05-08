
def matrix_subs(val, subs):
    """helper function to create a sympy matrix substitution table"""
    elem_subs = [ (val[row, col], subs[row, col])
                  for col in range(val.shape[1])
                  for row in range(val.shape[0]) ]
    return elem_subs + [(val, subs)]
