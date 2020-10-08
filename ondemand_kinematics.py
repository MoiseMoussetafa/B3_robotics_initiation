import math

# Dimensions used for the PhantomX robot :
constL1 = 51
constL2 = 63.7
constL3 = 93

# Angles corrections for real arm
theta2Correction = math.radians(20.69)
theta3Correction = math.radians(5.06)

# Dimensions used for the simple arm simulation
# bx = 0.07
# bz = 0.25
# constL1 = 0.085
# constL2 = 0.185
# constL3 = 0.250

def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    theta1 = math.radians(theta1)  
    theta2 = math.radians(theta2) 
    theta3 = math.radians(theta3) 

    #Corrections angles
    theta2 = theta2 + theta2Correction
    theta3 = theta3 - (math.pi/2) + theta3Correction + theta2Correction

    plan_contribution = l1+l2*math.cos(-theta2)+l3*math.cos(-theta2+theta3)

    x = plan_contribution*math.cos(theta1)
    y = plan_contribution*math.sin(theta1)
    z = l3*math.sin(theta3 - theta2) + l2*math.sin(-theta2)

    return [x, y, z]

def computeIK(x, y, z, l1=constL1, l2=constL2, l3=constL3):

    d13 = math.sqrt(x**2 + y**2) - l1
    d = math.sqrt(d13**2 + z**2)

    if (x==0):
        if (y>=0):
            theta1 = math.radians(90)
        if (y<0):
            theta1 = math.radians(-90)
    else:
        theta1 = math.atan(y/x)
    theta2 = math.atan(z/d13) + math.acos((l3**2 - l2**2 - d**2 ) /(-2*l2*d))
    theta3 = math.radians(180) - math.acos((d**2 - l2**2 - l3**2) / (-2*l2*l3))

    theta1 = math.degrees(theta1)
    theta2 = -(math.degrees(theta2) + math.degrees(theta2Correction))
    theta3 = -((math.degrees(theta3) 
        - math.degrees(math.pi/2) 
        + math.degrees(theta3Correction)
        + math.degrees(theta2Correction)))

    return (theta1, theta2, theta3)

def main():
    print("\nTesting the kinematic funtions (computeDK)...")
    t1 = float(input("theta1 : "))
    t2 = float(input("theta2 : "))
    t3 = float(input("theta3 : "))
    print(
        "Results : P3{}".format(computeDK(t1,t2,t3, l1=constL1, l2=constL2, l3=constL3)
        )
    )

    print("\n===================================================================")
    print("\nTesting the inverse kinematic funtions (computeIK)...")
    P3X = float(input("P3X : "))
    P3Y = float(input("P3Y : "))
    P3Z = float(input("P3Z : "))
    print(
        "Results : {}".format(computeIK(P3X,P3Y,P3Z, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()




if __name__ == "__main__":
    main()
