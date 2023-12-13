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

    # Normalize theta1 within the range (-180, 180) +++
    #if theta1 > 180:
    #    theta1 -= 360

    # Calculate the distance P from the origin (0, 0) to the point (x, y)
    P = math.sqrt(x ** 2 + y ** 2)

    # Calculate theta2 and theta3 using inverse trigonometric functions
    theta2 = math.atan2(z,P)
    theta3 = 0

    # Return the calculated angles rounded to 2 decimal places
    return round(theta1, 2), round(math.degrees(theta2), 2), round(theta3, 2) ,P




def plotUpdateX(val = 0):
    global save_for_run
    global ax
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
    ax.text(-10, -10, -10, f"Angle 1: {ang1}\nAngle 2: {ang2}\nAngle 3: {ang3}\nP: {P}\nCoordinate_end : {coord_end}" , color='black', fontsize=10)

    count = sum(bool(sublist) for sublist in save_for_run)
    print(f"coordinate point = {count}")

    if len(save_for_run) > 0:
        for i in range(count):  # ใช้ count ไม่ได้เพราะบางครั้ง count อาจจะมีค่าเกินไป
            if i < len(save_for_run):  # ตรวจสอบว่า index ที่ใช้ไม่เกินขนาดของ save_for_run
                ax.scatter(
                    save_for_run[i][0],
                    save_for_run[i][1],
                    save_for_run[i][2],
                    color='black',
                    s=50,
                    label=f'Saved Position {i}'
                )
                ax.text(save_for_run[i][0], save_for_run[i][1], save_for_run[i][2], f'{i}', color='red')
    else:
        print("No positions saved.")

    plt.draw()
    



def plotUpdateY(val = 0):
    global save_for_run
    global ax
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
    ax.text(-10, -10, -10, f"Angle 1: {ang1}\nAngle 2: {ang2}\nAngle 3: {ang3}\nP: {P}\nCoordinate_end : {coord_end}" , color='black', fontsize=10)
    
    count = sum(bool(sublist) for sublist in save_for_run)
    print(f"coordinate point = {count}")

    if len(save_for_run) > 0:
        for i in range(count):  # ใช้ count ไม่ได้เพราะบางครั้ง count อาจจะมีค่าเกินไป
            if i < len(save_for_run):  # ตรวจสอบว่า index ที่ใช้ไม่เกินขนาดของ save_for_run
                ax.scatter(
                    save_for_run[i][0],
                    save_for_run[i][1],
                    save_for_run[i][2],
                    color='black',
                    s=50,
                    label=f'Saved Position {i}'
                )
                ax.text(save_for_run[i][0], save_for_run[i][1], save_for_run[i][2], f'{i}', color='red')
    else:
        print("No positions saved.")

    plt.draw()
    


def plotUpdateZ(val = 0):
    global save_for_run
    global ax
    global coord_end
    coord_end[2] = val
    ax.clear()
    
    ang1, ang2, ang3 , P = inverseKinematics(coord_end)
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
    ax.text(-10, -10, -10, f"Angle 1: {ang1}\nAngle 2: {ang2}\nAngle 3: {ang3}\nP: {P}\nCoordinate_end : {coord_end}" , color='black', fontsize=10)    

    count = sum(bool(sublist) for sublist in save_for_run)
    print(f"coordinate point = {count}")

    # Check if there are enough points in save_for_run before accessing by index
    if len(save_for_run) > 0:
        for i in range(count):  # ใช้ count ไม่ได้เพราะบางครั้ง count อาจจะมีค่าเกินไป
            if i < len(save_for_run):  # ตรวจสอบว่า index ที่ใช้ไม่เกินขนาดของ save_for_run
                ax.scatter(
                    save_for_run[i][0],
                    save_for_run[i][1],
                    save_for_run[i][2],
                    color='black',
                    s=50,
                    label=f'Saved Position {i}'
                )
                ax.text(save_for_run[i][0], save_for_run[i][1], save_for_run[i][2], f'{i}', color='red')

    else:
        print("No positions saved.")

    plt.draw()

save_pos = [0,0,0]  # กำหนดให้เป็นลิสต์ขนาด 3 ตำแหน่ง ซึ่งจะใช้เก็บค่า coord_end ปัจจุบัน
save_for_run = [[] for _ in range(10)]  # สร้างลิสต์ขนาด 10 ตัว แต่ละตัวเป็นลิสต์เปล่าๆ

save_button_ax = plt.axes([0.1, 0.8, 0.04, 0.04])  # ตำแหน่งของปุ่ม Save
save_button = Button(save_button_ax, 'Save', color='lightgoldenrodyellow', hovercolor='0.975')

start_button_ax = plt.axes([0.05, 0.8, 0.04, 0.04])  # ตำแหน่งของปุ่ม Start
start_button = Button(start_button_ax, 'Start', color='lightgoldenrodyellow', hovercolor='0.975')

count = 0
line = 0
scatter_list = []
    
def save_position(event):
    global count
    global coord_end
    global line
    global save_pos

    if line < 10:
        with open('saved_position.txt', 'a') as file:
            file.write(f"Current position: {coord_end}\n")
            save_for_run[line] = list(coord_end)  # บันทึก coord_end ลงใน save_for_run ตาม line
            save_pos = list(coord_end)  # ปรับค่าของ save_pos เพื่อให้เก็บค่า coord_end ปัจจุบัน

        ax.scatter(coord_end[0], coord_end[1], coord_end[2], color='black', s=50, label='Saved Position')

        plt.text(-0.8, -1.5 + count, f"Saved coordinates {line}: {save_for_run[line]}", ha='left')

        count -= 0.5
        line += 1
        print(save_for_run)
        plt.draw()
    else :
        plt.text(-0.8, -1.5 + count, f"Exceeded maximum allowed saves (10) Please Reset Your Program", ha='left')
    return save_for_run

def start_button_clicked(event):
    global save_for_run
    global ax
    count = sum(bool(sublist) for sublist in save_for_run)
    i = 0
    print(f"coordinate point = {count}")
    move_to_saved_position()

def move_to_saved_position():
    global coord_end
    global save_for_run
    
    count = sum(bool(sublist) for sublist in save_for_run)
    if count >= 2:
        # Determine the step size for each coordinate axis
        step_size = 0.1  # Adjust this value according to the desired speed of movement
        tolerance = 0.01  # Set a tolerance for comparing coordinates
        
        for i in range(count - 1):  # Loop through save_for_run elements
            start_point = save_for_run[i]  # Starting point
            end_point = save_for_run[i + 1]  # Destination

            # Set coord_end to the starting position (save_for_run[i])
            coord_end = start_point + [0.1,0.1,0.1]

            # Update coord_end iteratively towards the destination
            while any(abs(coord_end[j] - end_point[j]) > tolerance for j in range(3)):
                for j in range(3):
                    if abs(coord_end[j] - end_point[j]) > tolerance:  # Compare the absolute difference with tolerance
                        if coord_end[j] < end_point[j]:
                            coord_end[j] += step_size
                        else:
                            coord_end[j] -= step_size

                # Update the plot for each step with orange color
                plotUpdateX(coord_end[0])
                plotUpdateY(coord_end[1])
                plotUpdateZ(coord_end[2])
                plt.pause(0.01)  # Adjust the pause duration as needed for visualization
    else:
        print("At least two positions need to be saved to initiate movement.")





start_button.on_clicked(start_button_clicked)
save_button.on_clicked(save_position)


sliderX.on_changed(plotUpdateX)
sliderY.on_changed(plotUpdateY)
sliderZ.on_changed(plotUpdateZ)
plt.show()