#!/usr/bin/python

# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project

# You can also use the new version of the API:
from robodk import *    # RoboDK API
from robolink import *

import sys 
import os
import re

class Pen:
    def __init__(self):
        RDK = Robolink()
        
        # get the robot, frame and tool objects
        self.robot = RDK.ItemUserPick('', ITEM_TYPE_ROBOT)
        self.Board_Frame = RDK.Item('Board_Frame')
        self.Pencil = RDK.Item('Pencil')

        # get the pixel reference to draw
        self.pixel = RDK.Item('pixel')

        image = RDK.Item('Display_Board')
        if image.Valid() and image.Type() == ITEM_TYPE_OBJECT: image.Delete()

        board_pixel = RDK.Item('Blackboard')
        board_pixel.Copy()
        self.board = self.Board_Frame.Paste()
        self.board.setVisible(True, False)
        self.board.setName('Display_Board')


    def position_to_pose(self, position, unit_vector):
        return transl(position.x, position.y, 0)*rotz(unit_vector.angle())

    def write(self, content):
        """Draws the image with the robot. It is slower that svg_draw_quick but it makes sure that the image can be drawn with the robot."""

        APPROACH = 100  # approach distance in MM for each path    
        # delete previous image if any

        home_joints = self.robot.JointsHome().tolist() #[0,0,0,0,90,0] # home joints, in deg
        if abs(home_joints[4]) < 5:
            home_joints[4] = 90.0

        self.robot.setPoseFrame(self.Board_Frame)
        self.robot.setPoseTool(self.Pencil)
        self.robot.MoveJ(home_joints)

        # get the target orientation depending on the tool orientation at home position
        T2T_Orientation = invH(self.Board_Frame.Pose())*self.robot.SolveFK(home_joints)*self.Pencil.Pose()
        T2T_Orientation[0:3,3] = Mat([0,0,0])

        size_img = content.size_poly()  # returns the size of the current polygon
        self.board.Scale([size_img.x/250, size_img.y/250, 1]) # adjust the board size to the image size (scale)

        for path in content:
            # use the pixel reference to set the path color, set pixel width and copy as a reference
            self.pixel.Recolor(path.fill_color)
            data_points_in_current_path = path.nPoints()

            # robot movement: approach to the first target
            initial_target = path.getPoint(0)
            target0 = transl(initial_target.x, initial_target.y, 0)*T2T_Orientation
            target0_app = target0*transl(0,0,-APPROACH)
            self.robot.MoveL(target0_app)

            for i in range(data_points_in_current_path):
                p_i = path.getPoint(i)
                v_i = path.getVector(i)       

                pt_pose = self.position_to_pose(p_i, v_i)
                
                target = transl(p_i.x, p_i.y, 0)*T2T_Orientation

                # Move the robot to the next target
                self.robot.MoveL(target)

                # create a new pixel object with the calculated pixel pose
                self.board.AddGeometry(self.pixel, pt_pose)

            target_app = target*transl(0,0,-APPROACH)
            self.robot.MoveL(target_app)

        self.robot.MoveL(home_joints)
        
        
# You can also use the new version of the API:
from svgpy.svg import *

SIZE_BOARD = [500, 1000]     # Size of the image. The image will be scaled keeping its aspect ratio
MM_X_PIXEL = 10             # in mm. The path will be cut depending on the pixel size. If this value is changed it is recommended to scale the pixel object

#--------------------------------------------------------------------------------
# Program start
# select the file to draw
# import the SVG file
svgdata = svg_load('test.svg')

IMAGE_SIZE = Point(SIZE_BOARD[0],SIZE_BOARD[1])   # size of the image in MM
svgdata.calc_polygon_fit(IMAGE_SIZE, MM_X_PIXEL)

writer = Pen()

# draw the image with the robot:
writer.write(svgdata)