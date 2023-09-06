

class Atmosphere: #Altitude 0

    def __init__(self) -> None:
        pass

    TempSea = 15
    Temp = 0  # Degree at 0km
    pressure = 101325  # Pa at 0km
    LapsRate = 0.0065
    gravity = 9.80665
    BoltzmanConstant = 1380649/16021766340
    MassOfDryAir = 28.9645  # g/mol
    GasConstant = 8314.32  # US Standard Atmosphere 1976 N*m*k*mol^-1*K^-1
    Rspecific = GasConstant / MassOfDryAir  # J⋅K^−1⋅mol^−1
    K = 273.15

    k = 1.4  # ratio of specific heats
    R = 286.9  # individual gas constant
