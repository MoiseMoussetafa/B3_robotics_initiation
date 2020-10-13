import math

# Dimensions used for the PhantomX robot :
constL1 = 51
constL2 = 63.7
constL3 = 93

# Angles corrections for real arm, with transformation of angles in radians
theta2Correction = math.radians(20.69)
theta3Correction = math.radians(5.06)

# Dimensions used for the simple arm simulation
# bx = 0.07
# bz = 0.25
# constL1 = 0.085
# constL2 = 0.185
# constL3 = 0.250

# Functions of Direct Kinematics
def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    #Transformations angles in radians
    theta1 = math.radians(theta1)  
    theta2 = math.radians(theta2) 
    theta3 = math.radians(theta3) 

    #Corrections angles
    theta2 = theta2 + theta2Correction
    theta3 = theta3 - (math.pi/2) + theta3Correction + theta2Correction

    #General formule used, and others formules for x, y, z
    plan_contribution = l1+l2*math.cos(-theta2)+l3*math.cos(-theta2+theta3)

    x = plan_contribution*math.cos(theta1)
    y = plan_contribution*math.sin(theta1)
    z = l3*math.sin(theta3 - theta2) + l2*math.sin(-theta2)

    #Return the results
    return [x, y, z]


# Functions of Inverse Kinematics
def computeIK(x, y, z, l1=constL1, l2=constL2, l3=constL3):

    #Prerequisite calculations
    d13 = math.sqrt(x**2 + y**2) - l1
    d = math.sqrt(d13**2 + z**2)

    #Loop "if" when x = 0, because it causes a division by 0 so it must be done differently
    if (x==0):
        if (y>=0):
            theta1 = math.radians(90)
        if (y<0):
            theta1 = math.radians(-90)
    
    # If we are not in the case of x = 0 
    else:
        theta1 = math.atan(y/x)

    # All others formules    
    theta2 = math.atan(z/d13) + math.acos((l3**2 - l2**2 - d**2 ) /(-2*l2*d))
    theta3 = math.radians(180) - math.acos((d**2 - l2**2 - l3**2) / (-2*l2*l3))

    theta1 = math.degrees(theta1)
    theta2 = -(math.degrees(theta2) + math.degrees(theta2Correction))
    theta3 = -((math.degrees(theta3) 
        - math.degrees(math.pi/2) 
        + math.degrees(theta3Correction)
        + math.degrees(theta2Correction)))

    #Return the results
    return (theta1, theta2, theta3)


#Main program, with text prints, function call, format of the result
def main():
    print("\nTesting the kinematic funtions (computeDK)...\n")
    print ("OK Test 1 - Values : 0°,0°,0° \n\tExpected : [118.79, 0.0, -115.14]")
    print(
        "\t Results : {}".format(computeDK(0,0,0, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()
    print ("OK Test 2 - Values : 90°,0°,0° \n\tExpected : [0.0, 118.79, -115.14]")
    print(
        "\t Results : {}".format(computeDK(90,0,0, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()
    print ("OK Test 3 - Values : 180°,-30.501°,-67.819° \n\tExpected : [-64.14, 0.0, -67.79]")
    print(
        "\t Results : {}".format(computeDK(180,-30.501,-67.79, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()
    print ("OK Test 4 - Values : 0°,-30.645°,38.501° \n\tExpected : [203.23, 0.0, -14.30]")
    print(
        "\t Results : {}".format(computeDK(0,-30.645,38.501, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print("\n===================================================================================")
    print("\nTesting the inverse kinematic funtions (computeIK)...\n")
    print ("OK Test 5 - Values : [118.79, 0.0, -115.14] \n\tExpected : (0°,0°,0°)")
    print(
        "\t Results : {}".format(computeIK(118.79,0.0,-115.14, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()
    print ("OK Test 6 - Values : [0.0, 118.79, -115.14] \n\tExpected : (90°,0°,0°)")
    print(
        "\t Results : {}".format(computeIK(0,118.79,-115.14, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()
    print ("OK Test 7 - Values : [-64.14, 0.0, -67.79] \n\tExpected : (180°,-30.501°,-67.819°)")
    print(
        "\t Results : {}".format(computeIK(-64.14, 0.0, -67.79, l1=constL1, l2=constL2, l3=constL3)
        )
    )
    print()
    print ("OK Test 8 - Values : [203.23, 0.0, -14.30] \n\tExpected : (0°,-30.645°,38.501°)")
    print(
        "\t Results : {}".format(computeIK(203.23, 0.0, -14.30, l1=constL1, l2=constL2, l3=constL3)
        )
    ) 
    print()

#Start code
if __name__ == "__main__":
    main()
