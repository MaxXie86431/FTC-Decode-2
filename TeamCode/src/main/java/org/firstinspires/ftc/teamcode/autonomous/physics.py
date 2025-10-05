import math
import math

def v_min_required(d, h, g):
    # Given (d, h), find minimum velocity to 
    # reach from a 2d span (0,0) -> (d, h) with g meaning gravity
    return math.sqrt(g * (h + math.sqrt(h**2 + d**2)))



def solve_theta(d, g, h, v):
    min_v = v_min_required(d, h, g)
    if v < min_v:
        print(f"Velocity [{v}] should be equal or higher than {min_v}")
        return None
    
    discriminant = d**2 - (2 * g * d**2 / v**2) * (h + (g * d**2) / (2 * v**2))
    
    if discriminant < 0:
        print("No real solution satisfying the equation")
        return None

    sqrt_term = math.sqrt(discriminant)
    denom = g * d**2 / v**2

    tan_theta1 = (d + sqrt_term) / denom
    tan_theta2 = (d - sqrt_term) / denom

    theta1 = math.atan(tan_theta1)
    theta2 = math.atan(tan_theta2)

    return theta1, theta2


d = 50.0   # meters
g = 9.80665   # m/s^2
h = 5.0    # meters
v = 30.0   # m/s

thetas = solve_theta(d, g, h, v)
if thetas:
    print(f"Theta 1: {math.degrees(thetas[0])}°")
    print(f"Theta 2: {math.degrees(thetas[1])}°")
else:
    print("No real solution for theta.")
