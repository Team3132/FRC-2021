  def process(self, inframe, outframe):
        inimg = inframe.get()

        # Start measuring image processing time:
        self.timer.start()
        
        # Convert input image to BGR24:
        imgbgr = jevois.convertToCvBGR(inimg)
        h, w, chans = imgbgr.shape

        # Get pre-allocated but blank output image which we will send over USB:
        outimg = outframe.get()
        outimg.require("output", w * 2, h + 12, jevois.V4L2_PIX_FMT_YUYV)
        jevois.paste(inimg, outimg, 0, 0)
        jevois.drawFilledRect(outimg, 0, h, outimg.width, outimg.height-h, jevois.YUYV.Black)

        # Let camera know we are done using the input image:
        inframe.done()

        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.loadCameraCalibration(w, h)

        imgbgr = cv2.undistort(imgbgr, self.camMatrix, self.distCoeffs, dst=None, newCameraMatrix = None)
        
        import libjevois as jevois
import cv2
import numpy as np
import math # for cos, sin, etc
import time

GOAL_WIDTH = 1.0098 # width in meters
GOAL_HEIGHT = 0.432 # height in meters

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240 # pixels
CAMERA_HEIGHT = 15 # center of camera height off the ground, in inches

CAMERA_FOV = 65 #horizontal

def deg_to_rad(degrees):
    return degrees * (math.pi / 180)

# converts radians to degrees
def rad_to_deg(radians):
    return radians * (180 / math.pi)

# sine function which uses degrees
def sin(degrees):
    return math.sin(deg_to_rad(degrees))

# cosine function which uses degrees
def cos(degrees):
    return math.cos(deg_to_rad(degrees))

# tan function which uses degrees
def tan(degrees):
    return math.tan(deg_to_rad(degrees))

# inverse sine function which returns degrees
def asin(ratio):
    return rad_to_deg(math.asin(ratio))

# inverse cosine function which returns degrees
def acos(ratio):
    return rad_to_deg(math.acos(ratio))

# inverse tan function which returns degrees
def atan(degrees):
    return rad_to_deg(math.atan(degrees))

class Corner():
    def __init__(self):
        self.xy = []
        self.score = -10000
    def update_score(self, X, Y, score):
        if score > self.score:
            self.xy = [X,Y]
            self.score = score

def cal_corners(contour):
    TL_corner = Corner()
    TR_corner = Corner()
    BL_corner = Corner()
    BR_corner = Corner()
    for point in contour:
        x = point[0][0] # +ve is more right
        y = point[0][1] # +ve is more down
        TL_corner.update_score(x, y, -x -y)
        TR_corner.update_score(x, y,  x -y)
        BL_corner.update_score(x, y, -x +y)
        BR_corner.update_score(x, y,  x +y)
    TL = TL_corner.xy
    TR = TR_corner.xy
    BL = BL_corner.xy
    BR = BR_corner.xy
    return TL, TR, BL, BR

def cal_goal_center_distance(distance):
    opposite = (98.25 - CAMERA_HEIGHT) * 0.0254
    hypotenuse = math.sqrt(pow(opposite,2) + pow(distance,2))
    return hypotenuse

def cal_point_distance(p, q):
    x = math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)
    return x


def cal_goal_skew(TL,TR,BL,BR, center_distance):
    # center_distance is the hypotenuse 
    HALF_WIDTH_GOAL = GOAL_WIDTH /2

    # left_side_length = cal_point_distance(TL, BL)
    # right_side_length = cal_point_distance(TR, BR)

    # left_side_distance = cal_distance(left_side_length) # this is wrong pls fix
    # left_acute_angle = cal_cosine_rule_deg(center_distance, HALF_WIDTH_GOAL, left_side_distance)

    # right_side_distance = cal_distance(right_side_length)
    # right_acute_angle = cal_cosine_rule_deg(center_distance, HALF_WIDTH_GOAL, right_side_distance)
    # avg_angle = (left_acute_angle + (180 - right_acute_angle))/2
    
    left_side_distance = cal_distance(TL[1])
    left_acute_angle = cal_cosine_rule_deg(center_distance, HALF_WIDTH_GOAL, left_side_distance)

    right_side_distance = cal_distance(TR[1])
    right_acute_angle = cal_cosine_rule_deg(center_distance, HALF_WIDTH_GOAL, right_side_distance)

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

def cal_distance(center_y): 
    # h1 is the top of the screen to the center of goal
    # h2 is center of goal to (floor + robot height)
    # h2 = 98.25" - camera height
        
    h2_pixels = (SCREEN_HEIGHT/2 - center_y) 
    h1_pixels = center_y
    
    h2_actual = (98.25 - CAMERA_HEIGHT) * 0.0254

    ratio = h2_actual / h2_pixels # metre / pixel

    h1_actual = h1_pixels * ratio

    distance = (h1_actual + h2_actual)/(tan(24.375))

    return distance

def cal_vert_dist_pixel_ratio(y):
    
    # h1 is the top of the screen to the center of goal
    # h2 is the  to (floor + robot height)
    # h2 = 98.25" - camera height
    h2_pixels = (SCREEN_HEIGHT/2 - y) 
    h2_actual = (98.25 - CAMERA_HEIGHT) *0.0254
    ratio = h2_actual / h2_pixels
    return ratio
    
def cal_hori_dist_pixel_ratio(vert_ratio):
    return vert_ratio * (4/3) # 4:3 aspect ratio

def cal_x_offset(mx, hori_ratio):
    x = mx - (SCREEN_WIDTH/2.0)
    return x * hori_ratio

# def cal_angle(pixel, distance):
#     angle = math.atan(pixel/distance)
#     angle = rad_to_deg(angle)
#     return angle

def cal_angle_test(x): # this works better, uses FOV 
    angle = ((x-(SCREEN_WIDTH/2))/(SCREEN_WIDTH/2))*(CAMERA_FOV/2)
    return angle

class FirstPython:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        # HSV color range to use:
        #
        # H: 0=red/do not use because of wraparound, 30=yellow, 45=light green, 60=green, 75=green cyan, 90=cyan,
        #      105=light blue, 120=blue, 135=purple, 150=pink
        # S: 0 for unsaturated (whitish discolored object) to 255 for fully saturated (solid color)
        # V: 0 for dark to 255 for maximally bright
               
        self.HSVmin = np.array([ 40,  20, 40], dtype=np.uint8)
        self.HSVmax = np.array([ 100, 255, 255], dtype=np.uint8)

        # Other processing parameters:
        self.epsilon = 0.015               # Shape smoothing factor (higher for smoother)
        self.hullarea = ( 15*15, 300*300 ) # Range of object area (in pixels) to track 
        self.hullfill = 35                 # Max fill ratio of the convex hull (percent)
        self.ethresh = 1500                 # Shape error threshold (lower is stricter for exact shape)
        self.margin = 5                    # Margin from from frame borders (pixels)
    
        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("FirstPython", 100, jevois.LOG_INFO)

        # CAUTION: The constructor is a time-critical code section. Taking too long here could upset USB timings and/or
        # video capture software running on the host computer. Only init the strict minimum here, and do not use OpenCV,
        # read files, etc
               
    # ###################################################################################################
    ## Parse a serial command forwarded to us by the JeVois Engine, return a string
    def parseSerial(self, str):
        jevois.LINFO("parseSerial received command [{}]".format(str))
        parts = str.split()
        try:
            if parts[0] == "setHSVMin":
                self.HSVmin, response = self.parseHSVValues(parts)
                return response
            if parts[0] == "setHSVMax":
                self.HSVmax, response = self.parseHSVValues(parts)
                return response
        except Exception as e:
            return "ERR {}".format(e)
        return "ERR Unsupported command {}".format(parts[0])

    def parseHSVValues(self, parts):
        if len(parts) != 4:
             raise Exception("Insufficient number of parameters for setHSV{Min|Max}, expected four")
        h = int(parts[1])
        s = int(parts[2])
        v = int(parts[3])
        return (np.array([h, s, v], dtype=np.uint8), "HSV min set to {} {} {}".format(h, s, v))
        

    # ###################################################################################################
    ## Load camera calibration from JeVois share directory
    def loadCameraCalibration(self, w, h):
        cpf = "/jevois/share/camera/calibration{}x{}.yaml".format(w, h)
        fs = cv2.FileStorage(cpf, cv2.FILE_STORAGE_READ)
        if (fs.isOpened()):
            self.camMatrix = fs.getNode("camera_matrix").mat()
            self.distCoeffs = fs.getNode("distortion_coefficients").mat()
            jevois.LINFO("Loaded camera calibration from {}".format(cpf))
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))

    
    # ###################################################################################################
    ## Detect objects within our HSV range
    # Do the following checks to ensure it's the correct shape: 
        # Hull is quadrilateral
        # Number of edges / vertices
        # Angle of lines
        # Top corners further apart than bottom corners
        # Area
        # Fill

    def detect(self, imgbgr, outimg = None):
        maxn = 5 # max number of objects we will consider
        h, w, chans = imgbgr.shape

        # Convert input image to HSV:
        imghsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)
        
        # Isolate pixels inside our desired HSV range:
        imgth = cv2.inRange(imghsv, self.HSVmin, self.HSVmax)
        maskValues = "H={}-{} S={}-{} V={}-{} ".format(self.HSVmin[0], self.HSVmax[0], self.HSVmin[1],
                                                self.HSVmax[1], self.HSVmin[2], self.HSVmax[2])    
        
        # Create structuring elements for morpho maths:
        if not hasattr(self, 'erodeElement'):
            self.erodeElement = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
            self.dilateElement = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
        
        # Apply morphological operations to cleanup the image noise:
        imgth = cv2.erode(imgth, self.erodeElement)
        imgth = cv2.dilate(imgth, self.dilateElement)
            
        imgth = cv2.medianBlur(imgth,3)

        # Detect objects by finding contours:
        contours, hierarchy = cv2.findContours(imgth, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        maskValues += "N={} ".format(len(contours))

        # Only consider the 5 biggest objects by area:
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[:maxn]
        bestHull = [] # best hull detection
        goalCriteria = ""
        bestgoalCriteria = ""
        
        # Identify the "good" objects:
        for c in contours:
            
            # Keep track of our best detection so far:
            if len(goalCriteria) > len(bestgoalCriteria): bestgoalCriteria = goalCriteria
            goalCriteria = ""

            # Compute contour area:
            area = cv2.contourArea(c, oriented = False)

            # Compute convex hull:
            rawhull = cv2.convexHull(c, clockwise = True)
            rawhullperi = cv2.arcLength(rawhull, closed = True)
            hull = cv2.approxPolyDP(rawhull, epsilon = self.epsilon * rawhullperi * 3.0, closed = True)

            # Is it the right shape?
            if (hull.shape != (4,1,2)): continue # 4 vertices for the rectangular convex outline (shows as a trapezoid)
            goalCriteria += "H" # Hull is quadrilateral
            
          
            huarea = cv2.contourArea(hull, oriented = False)
            if huarea < self.hullarea[0] or huarea > self.hullarea[1]: continue
            goalCriteria += "A" # Hull area ok
                      
            hufill = area / huarea * 100.0
            if hufill > self.hullfill: continue
            goalCriteria += "F" # Fill is ok
          
            # Check object shape:
            peri = cv2.arcLength(c, closed = True)
            approx = cv2.approxPolyDP(c, epsilon = 0.015 * peri, closed = True)

            for x in hull:
                 jevois.drawLine(outimg,int(x[0][0]),int(x[0][1]),int(x[0][0]),int(x[0][1]),1 , jevois.YUYV.LightGreen)
                      
            # Reject the shape if any of its vertices gets within the margin of the image bounds. This is to avoid
            # getting grossly incorrect 6D pose estimates as the shape starts getting truncated as it partially exits
            # the camera field of view:
            reject = 0
            for v in c:
                if v[0,0] < self.margin or v[0,0] >= w-self.margin or v[0,1] < self.margin or v[0,1] >= h-self.margin:
                   reject = 1
                   break
               
            if reject == 1: continue
            goalCriteria += "M" # Margin ok
           
            TL, TR, BL, BR = cal_corners(c)

            # check top/bottom
            top = cal_point_distance(TL,TR)
            bottom = cal_point_distance(BL,BR)   

            # check left & right angle
            lratio = (BL[0] - TL[0])/(BL[1] - TL[1])
            rratio = (BR[0] - TR[0])/(BR[1] - TR[1])

            if (top / bottom) < 1.3 or lratio > 0.6 or rratio < -1: continue
            goalCriteria += "R" #ratio is good  
       
            # This detection is a keeper:
            goalCriteria += " OK"
            bestHull = hull
            break
            
        TL,TR,BL,BR = cal_corners(bestHull)
        
        try:
            mx = int((TL[0]+TR[0])/2)
            my = int((TL[1]+TR[1])/2)
            distance = cal_distance(my)
            center_distance = cal_goal_center_distance(distance)
            skew = cal_goal_skew(TL,TR,BL,BR, center_distance)
            goalCriteria += "ground dist = " + str(distance)
            goalCriteria += "skew = " + str(skew)
        except:
            print("hi")
        
        # Display any results requested by the users:
        if outimg is not None and outimg.valid():
            if (outimg.width == w * 2): jevois.pasteGreyToYUYV(imgth, outimg, w, 0)
            jevois.writeText(outimg, maskValues + goalCriteria, 3, h+1, jevois.YUYV.White, jevois.Font.Font6x10)
            
        return bestHull
 
    # ###################################################################################################
    ## Send serial messages, one per object
    def sendAllSerial(self,found, distance, x):
        # time, found, distance, center of goal 
        now = time.time()
        
        jevois.sendSerial("D3 {} {} {} {} FIRST".
                            format(now,found, distance,x)) 
                              
    # ###################################################################################################
    ## Draw all detected objects in 3D
    def drawDetections(self, outimg, bestHull):
                   
        TL,TR,BL,BR = cal_corners(bestHull)

        try: 
            mx = int((TL[0]+TR[0])/2)
            my = int((TL[1]+TR[1])/2)
            jevois.drawLine(outimg,mx,my,mx,my,2 , jevois.YUYV.LightGrey)

        except:
            print("hi")
        
        # center of goal
        
    # ###################################################################################################
    ## Process function with no USB output
    def processNoUSB(self, inframe):
        # Get the next camera image (may block until it is captured) as OpenCV BGR:
        imgbgr = inframe.getCvBGR()
        h, w, chans = imgbgr.shape
        
        # Start measuring image processing time:
        self.timer.start()

        # Get a list of quadrilateral convex hulls for all good objects:
        bestHull = self.detect(imgbgr)

        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.loadCameraCalibration(w, h)

        # Send all serial messages:
        #self.sendAllSerial(w, h, bestHull, rvecs, tvecs)

        # Log frames/s info (will go to serlog serial port, default is None):
        self.timer.stop()

    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe):
        # Get the next camera image (may block until it is captured). To avoid wasting much time assembling a composite
        # output image with multiple panels by concatenating numpy arrays, in this module we use raw YUYV images and
        # fast paste and draw operations provided by JeVois on those images:
        inimg = inframe.get()

        # Start measuring image processing time:
        self.timer.start()
        
        # Convert input image to BGR24:
        imgbgr = jevois.convertToCvBGR(inimg)
        h, w, chans = imgbgr.shape

        # Get pre-allocated but blank output image which we will send over USB:
        outimg = outframe.get()
        outimg.require("output", w * 2, h + 12, jevois.V4L2_PIX_FMT_YUYV)
        jevois.paste(inimg, outimg, 0, 0)
        jevois.drawFilledRect(outimg, 0, h, outimg.width, outimg.height-h, jevois.YUYV.Black)

        # Let camera know we are done using the input image:
        inframe.done()
        
        # Get a list of quadrilateral convex hulls for all good objects:
        # bestHull = self.detect(imgbgr, outimg)

        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.loadCameraCalibration(w, h)

        imgbgr = cv2.undistort(imgbgr, self.camMatrix, self.distCoeffs, dst=None, newCameraMatrix = None)
        
        # Get a list of quadrilateral convex hulls for all good objects:
        bestHull = self.detect(imgbgr, outimg)

        found = False
        distance = 0
        angle = 0

        try: 
            if(bestHull.any()):
                found = True
            TL,TR,BL,BR = cal_corners(bestHull)
        
            mx = int((TL[0]+TR[0])/2)
            my = int((TL[1]+TR[1])/2)
                        
            distance = cal_distance(my)
            p = cal_vert_dist_pixel_ratio(my)
            cal_hori = cal_hori_dist_pixel_ratio(p)
            xOffset = cal_x_offset(mx, cal_hori)
            # angle = cal_angle(xOffset,distance) # calculated based on pixel:distance ratio
            angle = cal_angle_test(mx) # calculated based on FOV, works better 

        except:
            print("target not found")
        
        # Send all serial messages:
        self.sendAllSerial(found, distance, angle)
        
        #cv2.drawContours(outimg, [foundcontours], 0, (255, 0, 0), 2)
    
        # Draw all detections in 3D:
        self.drawDetections(outimg, bestHull)

        # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        fps = self.timer.stop()
        jevois.writeText(outimg, fps, 3, h-10, jevois.YUYV.White, jevois.Font.Font6x10)
    
        # We are done with the output, ready to send it to host over USB:
        outframe.send()