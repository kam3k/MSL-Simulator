from math import sqrt

def find_intersection(line_1, line_2):
    """Finds the intersection point (x, y) of two infinitely long lines. The 
    lines input to this function are dictionaries with keys a, b, and c that
    describe the line in the form ax + by = c."""
    det = line_1['a'] * line_2['b'] - line_2['a'] * line_1['b']
    # Lines are parallel if the determinant is zero
    if det == 0:
        return None
    x = (line_2['b'] * line_1['c'] - line_1['b'] * line_2['c']) / float(det)
    y = (line_1['a'] * line_2['c'] - line_2['a'] * line_1['c']) / float(det)
    return (x, y)

def validate_intersection(line, point):
    """Determines whether or not a point that is known to be on a line is on a 
    particular segment of that line. The line dictionary has the endpoints of
    the line segment (keys x_1, y_1, x_2, and y_2)."""
    x, y = point
    if (min(line['x_1'], line['x_2']) <= x <= max(line['x_1'], line['x_2']) and
        min(line['y_1'], line['y_2']) <= y <= max(line['y_1'], line['y_2'])):
        return True
    else:
        return False

def dist_between_points(point_1, point_2):
    """Returns the absolute distance between two points."""
    x_1, y_1 = point_1
    x_2, y_2 = point_2
    return sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)

def dist_point_to_line(line, point):
    """Returns the minimum distance between a point and a line."""
    x, y = point
    return ( (line['a'] * x + line['b'] * y - line['c']) / 
            sqrt(line['a']**2 + line['b']**2))

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
    
