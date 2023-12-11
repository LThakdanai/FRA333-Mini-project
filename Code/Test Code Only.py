# In this file I'll try to implement a simple inverse kinematics exaple and simulate it in a 3D plot
# 
# 
# 
# the coordinates are defined as [x, y, z]

import math
from ntpath import join
from turtle import color
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from matplotlib.widgets import Button


# The home coordinates will be [0,0,0]
coord_home = [0,0,0]
coord_end = [0,0,0]

len_coxa = 5
len_femur = 10
len_tibia = 10

joint1 = 0
joint2 = 0
joint3 = 0

point1 = [0,0,0]
point2 = [0,-5,0]
point3 = [0,0,0]






fig = plt.figure(figsize=(4,4))

ax = fig.add_subplot(111, projection='3d')

axSlider = plt.axes([0.2, 0.1, 0.65, 0.03] )
aySlider = plt.axes([0.2, 0.065, 0.65, 0.03] )
azSlider = plt.axes([0.2, 0.03, 0.65, 0.03] )
sliderX = Slider(axSlider, 'X', -10, 10, valinit=0, valstep = 0.1, color = 'red')
sliderY = Slider(aySlider, 'Y', -10, 10, valinit=0, valstep = 0.1, color = 'blue')
sliderZ = Slider(azSlider, 'Z', -10, 10, valinit=0, valstep = 0.1, color = 'green')



def forwardKinematics(ang1, ang2, ang3):
    print("Angle 1: ", ang1,"Angle 2: ", ang2,"Angle 3: ", ang3 )
    # point 1
    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point1[0] = trans[(0,0)]
    point1[1] = trans[(1,0)]
    point1[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point1[0],point1[1],point1[2],1]).transpose())
    
    point1[0] = rot[(0,0)]
    point1[1] = rot[(1,0)]
    point1[2] = rot[(2,0)]


    # point 2
    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point2[0] = trans[(0,0)]
    point2[1] = trans[(1,0)]
    point2[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    
    point2[0] = rot[(0,0)]
    point2[1] = rot[(1,0)]
    point2[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    point2[0] = trans[(0,0)]
    point2[1] = trans[(1,0)]
    point2[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    
    point2[0] = rot[(0,0)]
    point2[1] = rot[(1,0)]
    point2[2] = rot[(2,0)]


    # point 3
    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[1,0,0,0],
                        [0,math.cos(math.radians(ang3)),-math.sin(math.radians(ang3)),0],
                        [0,math.sin(math.radians(ang3)),math.cos(math.radians(ang3)),0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())

    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)] 
    
    trans = np.dot(np.matrix([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]


    return point1, point2, point3


def coord2deg(x, y): 
    if x >= 0 and y >= 0:  
        angle = math.degrees(math.atan(y/x))
    elif x < 0 and y >= 0:  
        angle = 180 + math.degrees(math.atan(y/x))
    elif x < 0 and y < 0:   
        angle = 180 + math.degrees(math.atan(y/x))
    elif x >= 0 and y < 0:  
        angle = 360 + math.degrees(math.atan(y/x))
    return round(angle,2)


def inverseKinematics(pos):
    # Extract x, y, z coordinates from the input position
    x, y, z = pos[0], pos[1], pos[2]

    # To avoid potential zero-division error, add a very small value to x
    x += 0.0000001

    # Calculate theta1 using coord2deg function; represents angle from x-axis in anticlockwise rotation
    theta1 = coord2deg(x, y)

    # Remove offset due to the length of coxa from x and y coordinates
    x -= 0 * math.cos(math.radians(theta1))
    y -= 0 * math.sin(math.radians(theta1))

    # Normalize theta1 within the range (-180, 180) +++
    if theta1 > 180:
        theta1 -= 360

    # Calculate the distance P from the origin (0, 0) to the point (x, y)
    P = math.sqrt(x ** 2 + y ** 2)

    # Check if the distance from the origin to the target point is within reachable distance
    if math.sqrt(x ** 2 + y ** 2 + z ** 2) > len_femur + len_tibia:
        print("MATH ERROR: Coordinate is too far")
        # If out of reach, return theta1 and zeros for theta2 and theta3
        return [theta1, 0, 0]

    # Calculate alpha angle using arctan of z/P
    alpha = math.atan(z / P)

    # Calculate distance c using Pythagorean theorem
    c = math.sqrt(P ** 2 + z ** 2)

    # Calculate beta angle using law of cosines
    beta = math.acos((len_femur ** 2 + c ** 2 - len_tibia ** 2) / (2 * len_femur * c))

    # Calculate theta2 and theta3 using inverse trigonometric functions
    theta2 = math.atan2(z,P)
    theta3 = 0

    # Return the calculated angles rounded to 2 decimal places
    return round(theta1, 2), round(math.degrees(theta2), 2), round(theta3, 2) ,P




def plotUpdateX(val = 0):
    global coord_end
    coord_end[0] = val
    ax.clear()

    ang1, ang2, ang3, P = inverseKinematics(coord_end)
    point1, point2, point3 = forwardKinematics(ang1, ang2, ang3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([coord_end[0],coord_end[0]+1],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]], color='red')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]+1],[coord_end[2],coord_end[2]], color='blue')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]+1], color='green')

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

    ax.plot([coord_home[0], coord_end[0]], [coord_home[1], coord_end[1]], [coord_home[2], coord_end[2]], color='black')
    ax.text(-10, -10, -10, f"Angle 1: {ang1}\nAngle 2: {ang2}\nAngle 3: {ang3}\nP: {P}", color='black', fontsize=10)


def plotUpdateY(val = 0):
    global coord_end
    coord_end[1] = val
    ax.clear()

    ang1, ang2, ang3, P = inverseKinematics(coord_end)
    point1, point2, point3 = forwardKinematics(ang1, ang2, ang3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([coord_end[0],coord_end[0]+1],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]], color='red')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]+1],[coord_end[2],coord_end[2]], color='blue')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]+1], color='green')

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

    ax.plot([coord_home[0], coord_end[0]], [coord_home[1], coord_end[1]], [coord_home[2], coord_end[2]], color='black')


    
    ax.text(-10, -10, -10, f"Angle 1: {ang1}\nAngle 2: {ang2}\nAngle 3: {ang3}\nP: {P}", color='black', fontsize=10)


def plotUpdateZ(val = 0):
    global coord_end
    coord_end[2] = val
    ax.clear()

    ang1, ang2, ang3 , P= inverseKinematics(coord_end)
    point1, point2, point3 = forwardKinematics(ang1, ang2, ang3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([coord_end[0],coord_end[0]+1],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]], color='red')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]+1],[coord_end[2],coord_end[2]], color='blue')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]+1], color='green')

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

    

    ax.plot([coord_home[0], coord_end[0]], [coord_home[1], coord_end[1]], [coord_home[2], coord_end[2]], color='black')
   
    ax.plot([coord_home[0], coord_end[0]], [coord_home[1], coord_end[1]], [coord_home[2], coord_end[2]], color='red')
    ax.plot([coord_home[0], coord_end[0]], [coord_home[1], coord_end[1]], [coord_home[2], coord_end[2]], color='green')
    ax.plot([coord_home[0], coord_end[0]], [coord_home[1], coord_end[1]], [coord_home[2], coord_end[2]], color='blue')


    ax.text(-10, -10, -10, f"Angle 1: {ang1}\nAngle 2: {ang2}\nAngle 3: {ang3}\nP: {P}", color='black', fontsize=10)
    

position = []
save_button_ax = plt.axes([0.8, 0.01, 0.1, 0.04])  # ตำแหน่งของปุ่ม Save
save_button = Button(save_button_ax, 'Save', color='lightgoldenrodyellow', hovercolor='0.975')

def save_position(event):
    global coord_end
    # ทำการบันทึกค่า coord_end ที่ได้ปัจจุบัน
    with open('saved_position.txt', 'w') as file:
        file.write(f"Current position: {coord_end}")
        position = coord_end
    ax.scatter(coord_end[0], coord_end[1], coord_end[2], color='black', s=50, label='Saved Position')
    plt.legend()  # เพิ่มคำอธิบาย (legend) ลงในกราฟ
    plt.draw()  # เพื่อให้กราฟทำการรีเฟรชเพื่อแสดงจุดใหม่
    print(f"Saved position:",position)

save_button.on_clicked(save_position)


sliderX.on_changed(plotUpdateX)
sliderY.on_changed(plotUpdateY)
sliderZ.on_changed(plotUpdateZ)
plt.show()