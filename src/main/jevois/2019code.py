import cv2
import numpy as np
import math
import time
from angle_util import *

VERTICAL_FOV_LIFECAM = 33.58
HORIZONTAL_FOV_LIFECAM = 59.7
GOAL_HEIGHT_RELATIVE = 84 - 19.4 # vertical distance between the camera and the goal
CAMERA_ANGLE_OF_ELEVATION = 30
PIXELS_PER_DEGREE = 0.171/1.8
AIM_CORRECTION = 0 # 7 # The camera maybe off-centered with the robot

# The class the GoalDetector returns.
class Result:
    found = False
    contour = None
    area = 0
    corners = [[0,0],[0,0],[0,0],[0,0]]
    corner_area = 0
    avg_width = 0
    avg_height = [0,0]
    image_size = []
    goal_center = [0,0]
    aspect_ratio = 1
    aim = 0
    distance = 0
    skew = 0
    
    def toRioString(self, timestamp):
        # Convert to something the VisionSubsystem can understand.
        # Line format: <found>,<degrees>,<distance>,<skew>,<secsAgo>\n
        now = time.time()
        return "%d,%f,%f,%f\n" % (self.found, self.distance, self.skew, now - timestamp)

# returns the height and width of an image
def cal_image_size(image):
    #height, width, channels = image.shape
    height, width = image.shape
    return (width,height)

# applies a hsv mask to an image 
def mask_image(image, mask_values):
    if image is None:
        print("Error Trying to convert a null image")
        return image
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lc = np.array([mask_values[0], mask_values[1], mask_values[2]])
    hc = np.array([mask_values[3], mask_values[4], mask_values[5]])
    image_mask = cv2.inRange(image_hsv, lc, hc)
    return image_mask
    
# changes the resolution of an image and converts it to grayscale
# used to improve framerate when streaming back to the driver station
def resize_gray_image(image, factor):
    image = cv2.resize(image, (0,0), fx=factor, fy=factor)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image

# Tracks score and xy coordinate
class Corner():
    def __init__(self):
        self.xy = []
        self.score = -10000
    def update_score(self, X, Y, score):
        if score > self.score:
            self.xy = [X,Y]
            self.score = score

# Finds outermost contours (edges between black and white on a masked image)
def find_contours(image_mask):
    ret, thresh = cv2.threshold(image_mask, 0, 255, 0)
    img, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Change cv2.RETR_EXTERNAL to cv2.RETR_TREE to find all contours
    return contours

# takes a contour and finds the four outermost corners
# returns their coordinates in an array
def cal_corners(contour):
    TL_corner = Corner()
    TR_corner = Corner()
    BL_corner = Corner()
    BR_corner = Corner()
    # go through each point and see if it is a better corner then the last
    for point in contour:
        x = point[0][0] # +ve is more right.
        y = point[0][1] # +ve is more down
        TL_corner.update_score(x, y, -x -y)
        TR_corner.update_score(x, y, +x -y)
        BL_corner.update_score(x, y, -x +y)
        BR_corner.update_score(x, y, +x +y)
    TL = TL_corner.xy
    TR = TR_corner.xy
    BL = BL_corner.xy
    BR = BR_corner.xy
    return [TL, TR, BL, BR]

# calculates the area inside a contour
def cal_contour_area(contour):
    area = cv2.contourArea(contour)
    return area

# calculates the area inside of four corners
def cal_corner_area(corners):
    TL = corners[0]
    TR = corners[1]
    BL = corners[2]
    BR = corners[3]
    corner_contour = np.array([[corners[0], corners[2], corners[3], corners[1]]])
    corner_area = cv2.contourArea(corner_contour)
    return corner_area

# calculates the distance between two points using Pythagoras' Theorm
def cal_point_distance(p, q):
    c = math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)
    return c

# calculates the skew of the target (left_height/right_height)
def cal_goal_skew(corners, center_distance, distance_factor):
    HALF_WIDTH_GEAR_TARGET = 5.125 # IRL in inches

    TL = corners[0]
    TR = corners[1]
    BL = corners[2]
    BR = corners[3]
    left_side_length = cal_point_distance(TL, BL)
    right_side_length = cal_point_distance(TR, BR)

    left_side_distance = cal_distance(left_side_length, distance_factor)
    left_acute_angle = cal_cosine_rule_deg(center_distance, HALF_WIDTH_GEAR_TARGET, left_side_distance)

    right_side_distance = cal_distance(left_side_length, distance_factor)
    right_acute_angle = cal_cosine_rule_deg(center_distance, HALF_WIDTH_GEAR_TARGET, right_side_distance)
    avg_angle = (left_acute_angle + (180 - right_acute_angle))/2
    return 90 - avg_angle

# calculates the angle in a triangle of known lengths
# adj1 and adj2 are the adjacent sides to the angle
# opposite is the opposite angle to the the desired angle
def cal_cosine_rule_deg(adj1, adj2, opposite):
    adj1_sqrd = math.pow(adj1, 2)
    adj2_sqrd = math.pow(adj2, 2)
    opposite_sqrd = math.pow(opposite, 2)
    if (abs((adj1_sqrd + adj2_sqrd - opposite_sqrd)/(2*adj1*adj2)) >1 ):
        #print "error in cal_cosine_rule_deg() opposite_sqd = %f" %opposite_sqrd
        return 0
    return acos((adj1_sqrd + adj2_sqrd - opposite_sqrd)/(2*adj1*adj2))


# calculates the length of each side then the average width and height
def cal_avg_height_width(corners):
    TL = corners[0]
    TR = corners[1]
    BL = corners[2]
    BR = corners[3]
    top_side_length = cal_point_distance(TL, TR)
    bot_side_length = cal_point_distance(BL, BR)
    left_side_length = cal_point_distance(TL, BL)
    right_side_length = cal_point_distance(TR, BR)
    avg_height = (left_side_length + right_side_length)/2
    avg_width = (top_side_length + bot_side_length)/2
    return avg_width, avg_height

# calculates the center of the goal assuming it is a rectangle
def cal_goal_center(corners):
    [TL, TR, BL, BR] = corners
    goal_center = ((TL[0] + TR[0] + BL[0] + BR[0]) / 4, (TL[1] + TR[1] + BL[1] + BR[1]) / 4)
    return goal_center

# calculates the aspect ratio (width/height)
def cal_aspect_ratio(avg_width, avg_height):
    aspect_ratio = avg_width/avg_height
    return aspect_ratio

# Calculates distance to the goal based on the pixel height and real height
def cal_distance(dependent_var, factor):
    """Use this code to calibrate factor
    1.Change "DISTANCE" to the camera's current distance away from the goal
    2.Run program and average the first ten console outputs
        
    DISTANCE = 82
    FACTOR = DISTANCE*dependent_var
    print "%d" %FACTOR
    """    
    distance = round((factor/dependent_var), 1)
    #print(distance)
    return distance

# calculates the angle between the camera and the target
def cal_distance_position(image_size, goal_center):
    image_center = image_size[1]/2
    image_height = image_size[1]
    
    degrees_per_pixel = VERTICAL_FOV_LIFECAM/image_height
    pixel_height = image_size[1] - goal_center[1]

    elevation_angle = degrees_per_pixel * pixel_height - (VERTICAL_FOV_LIFECAM/2) + CAMERA_ANGLE_OF_ELEVATION
    horizontal_distance = (GOAL_HEIGHT_RELATIVE * sin(90 - elevation_angle))/sin(elevation_angle)
    
    # linear correction
    """
    Use Spreadsheet to calculate gradient and intercept
    1) Reset the current magic values to 0 and comment out the Pythagorus's theorm calculation just before the return
    2) Take readings of what the vision thinks the distance is and the real horizontal distance
    3) Input these into the following spreadsheet:
    https://docs.google.com/spreadsheets/d/1dRjTlxUB827p9uOyVD0SimTnsq3S977NRUhrfPGv-64/edit?usp=sharing
    """
    CORRECTION_GRADIENT = 0.431
    CORRECTION_INTERCEPT = -23.807
    horizontal_distance += horizontal_distance*CORRECTION_GRADIENT
    horizontal_distance += CORRECTION_INTERCEPT

    # Use pythagorus's theorm to find the distance between the camera and the goal
    # distance = cal_point_distance((horizontal_distance, GOAL_HEIGHT_RELATIVE),(0,0))
    return distance

# calculates the angle between the camera and the target
def cal_aim(image_size, goal_center):
    image_center = image_size[0]/2
    image_width = image_size[0]
    aim = (HORIZONTAL_FOV_LIFECAM/image_width * (goal_center[0] - image_center)) - AIM_CORRECTION # the camera may not be centered on the robot
    return aim


def annotate_image(image, fps, result, high_detail):
    # Always draw the center of the image and the fps
    draw_image_center(image, result.image_size)
    draw_fps(image, fps)
    
    if not result.found:
        return
    
    draw_goal_center(image, result.goal_center, (0,255,255), 15)
    draw_aim_distance(image, result.aim, result.distance)
    
    # Draw extra details if someone is viewing the images.
    if high_detail:
        draw_areas(image, result.contour, result.area, result.corner_area, result.corners)
        draw_aspect_ratio(image, result.aspect_ratio, result.goal_center)
        draw_rectangle_offset(image, result.corners, 5) 


# draws a rectangle from four specified points applying an offset of 5 pixels
def draw_rectangle_offset(image, corners, offset):
    TL = corners[0]
    TR = corners[1]
    BL = corners[2]
    BR = corners[3]
    cv2.line(image, (TL[0]-offset,TL[1]-offset), (TR[0]+offset,TR[1]-offset), (0,0,255), 2)
    cv2.line(image, (TR[0]+offset,TR[1]-offset), (BR[0]+offset,BR[1]+offset), (0,0,255), 2)
    cv2.line(image, (BR[0]+offset,BR[1]+offset), (BL[0]-offset,BL[1]+offset), (0,0,255), 2)
    cv2.line(image, (BL[0]-offset,BL[1]+offset), (TL[0]-offset,TL[1]-offset), (0,0,255), 2)

# draws the area, corner area and their values
def draw_areas(image, contour, area, corner_area, corners):
    cv2.drawContours(image, [contour], 0, (255,255,255), 2)
    draw_rectangle_offset(image, corners, 5)
    draw_text(image, str(area), corners[2], (0,150,0))
    draw_text(image, str(corner_area), corners[1], (0,0,150))

# draws a dot at specified coordinates
def draw_goal_center(image, goal_center, colour = (255,0,255), size = 8):
    cv2.line(image, goal_center, goal_center, colour, size)

# draws a vertical line taking into account for aim correction
def draw_image_center(image, image_size):
    x = int(image_size[0]/2 + AIM_CORRECTION/PIXELS_PER_DEGREE)
    cv2.line(image, (x,0), (x, image_size[1]), (0,255,255), 2)

# draws a label for the aspect ratio at goal_center
def draw_aspect_ratio(image, aspect_ratio, goal_center):
    draw_text(image, str(round(aspect_ratio,3)), goal_center, (255,0,255))

# draws text showing information on where the target is relative to the robot
def draw_aim_distance(image, aim, distance):
    text = "Distance: ~" + "%02.1f" %distance + "  Aim: ~" + "%02.1f" %aim
    draw_text(image, text, (0,25), (255,255,255))

# draws information on how quickly the script is running
def draw_fps(image, fps):
    text = " FPS: ~%02.1f" % fps
    cv2.putText(image, text, (0,50), cv2.FONT_HERSHEY_SIMPLEX, .75, (255,255,255), 2)

# draws text (called by other draw functions)
def draw_text(image, text, xy, colour=(0,0,255)):
    cv2.putText(image, text, (xy[0], xy[1]), cv2.FONT_HERSHEY_SIMPLEX, .75, colour, 2)