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
    x = P3X
    y = P3Y
    z = P3Z

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

# Direct Kinematics - Choose Theta used for DK
#t1 = input("DK Theta 1 in degrees : ")
#t1 = float(t1)
#t2 = input("DK Theta 2 in degrees : ")
#t2 = float(t2)
#t3 = input("DK Theta 3 in degrees : ")
#t3 = float(t3)

# Inverse Kinematics - Choose Positions used for IK
P3X = 0  
P3Y = 0
P3Z = 0

def main():
    print("Testing the kinematic funtions (computeDK)...")
    print ("OK Test 1 - Values : 0°,0°,0° \n\tExpected : [118.79, 0.0, -115.14]")
    print(
        "\t Results : {}".format(computeDK(0,0,0, 
        l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()
    print("Testing the inverse kinematic funtions (computeIK)...")
    print(
        "\t P3[{0} ; {1} ; {2}] --> {3}".format(P3X, P3Y, P3Z,
            computeIK(P3X, P3Y, P3Z, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()


if __name__ == "__main__":
    main()
