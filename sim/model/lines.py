def get_line_dict(x_1, y_1, x_2, y_2):
    """Returns a dictionary with the line's endpoints, as well as its linear
    coefficients ax + by = c."""
    line_dict = {'x_1': x_1, 
                 'y_1': y_1,
                 'x_2': x_2,
                 'y_2': y_2,
                 'a': y_2 - y_1,
                 'b': x_1 - x_2,
                 'c': (y_2 - y_1) * x_1 + (x_1 - x_2) * y_1}
    return line_dict
    
