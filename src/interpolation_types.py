from enum import Enum
 
class InterpolationTypes(Enum):
    Linear = 1
    Polynomial = 2
    Spline = 3
    Mimetic = 4