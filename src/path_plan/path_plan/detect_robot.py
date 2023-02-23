import cv2
import numpy as np

from .features import ret_smallest_obj,ret_largest_obj
from . import robot_setup
class find_robot():

    def __init__(self):

       
        self.is_bg_extracted =False

      
        self.background_model = [] #to hold background model
        self.env_og = []
        self.loc_robot = 0

        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.transform_arr = []

        self.orig_rot = 0
        self.rot_mat = 0


    @staticmethod
    # This function calculates the convex hull of the contours of the current frame  --> gives us whole env and with robot
    def ret_rois_boundinghull(rois_mask,cnts):
    # Create a black image with same size as rois_mask
        environmet_enclosure = np.zeros_like(rois_mask)
        if cnts:
            cnts_ = np.concatenate(cnts)
            cnts_ = np.array(cnts_)
            cv2.fillConvexPoly(environmet_enclosure, cnts_, 255)
        cnts_largest = cv2.findContours(environmet_enclosure, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        hull = cv2.convexHull(cnts_largest[0])
        cv2.drawContours(environmet_enclosure, [hull], 0, 255)
        return hull
    
    def update_frameofrefrence_parameters(self,X,Y,W,H,rot_angle):
        self.orig_X = X; self.orig_Y = Y; self.orig_rows = H; self.orig_cols = W; self.orig_rot = rot_angle # 90 degree counterClockwise
        self.transform_arr = [X,Y,W,H]
        # Rotation Matrix
        cos_deg = np.cos(np.deg2rad(self.orig_rot))
        sin_deg = np.sin(np.deg2rad(self.orig_rot))
        
        self.rot_mat = np.array([[cos_deg, sin_deg], [-sin_deg, cos_deg]])
        self.rot_mat_rev = np.array([[cos_deg, -sin_deg], [sin_deg, cos_deg]])

    @staticmethod
    def connect_cnts(bin_img):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        return(cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel))
    
    def extract_bg(self,frame):

        # a) Find Contours in frozen sat_view: our interest area is env and the robot
#so need to extract the mask of these areas --> we use edge detection alg
        edges = cv2.Canny(frame, 60, 150,None,3)
        # connect contours
        edges = self.connect_cnts(edges)
        cnts = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0] #getting external contours of all the edges
        rois_mask = np.zeros((frame.shape[0],frame.shape[1]),dtype= np.uint8)
        for idx,_ in enumerate(cnts): #loop over all the contours to draw them 
            cv2.drawContours(rois_mask, cnts, idx, 255,-1)

        # b) extract background model by first --> remove the smallest object which is robot, second --> fill the empty area w ground replica

        min_cntr_idx = ret_smallest_obj(cnts)
        rois_NoRobot_msk = rois_mask.copy()
        #    If Smallest Object found         
        if min_cntr_idx !=-1:
            cv2.drawContours(rois_NoRobot_msk, cnts, min_cntr_idx, 0,-1)
            # Drawing dilated robot_mask
            robot_mask = np.zeros_like(rois_mask)
            cv2.drawContours(robot_mask, cnts, min_cntr_idx, 255,-1)
            cv2.drawContours(robot_mask, cnts, min_cntr_idx, 255, 3)
            notrobot_mask = cv2.bitwise_not(robot_mask) #region w no robot in it 
            frame_robot_remvd = cv2.bitwise_and(frame, frame,mask = notrobot_mask)
            # Create ground replica to fill empty region
            base_clr = frame_robot_remvd[0][0]
            Ground_replica = np.ones_like(frame)*base_clr
            # Create background model
            self.background_model = cv2.bitwise_and(Ground_replica, Ground_replica,mask = robot_mask)
            self.background_model = cv2.bitwise_or(self.background_model, frame_robot_remvd) # background extraction is one
        
        
        # Step 2: Extracting the environment
        
        # a) Finding dimensions of hull enclosing largest contour
        hull = self.ret_rois_boundinghull(rois_mask,cnts)
        [X,Y,W,H] = cv2.boundingRect(hull)
        # b) Cropping robot_mask from the image
        env = rois_NoRobot_msk[Y:Y+H,X:X+W]
        env_occupencygrid = cv2.bitwise_not(env) #get OG where robot can travel by inverting the maze --> it is for mapping 
        self.env_og = cv2.rotate(env_occupencygrid, cv2.ROTATE_90_COUNTERCLOCKWISE) #entry on top

        # Storing Crop and Rot Parameters required to maintain frame of refrence in the orig image
        self.update_frameofrefrence_parameters(X,Y,W,H,90)

        if (robot_setup.debug and robot_setup.debug_localization):   #for GUI
            cv2.imshow("1. Mask of Interest Areas",rois_mask)
            cv2.imshow("2. Robot Frame Removed",frame_robot_remvd)
            cv2.imshow("3. Ground Replica",Ground_replica)
            cv2.imshow("4. Background Replica",self.background_model)
            cv2.imshow("5. Environment OG",self.env_og)

    @staticmethod
    def get_centroid(cnt):
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cy,cx)

    def get_rbt_loc(self,rbt_cnt,robot_mask):
        
        # a) Get the centroid of the rbt
        bot_cntr = self.get_centroid(rbt_cnt)
        # b) Converting from point --> array to apply transforms
        bot_cntr_arr =  np.array([bot_cntr[1],bot_cntr[0]])
        # c) Shift origin from sat_view -> env
        bot_cntr_translated = np.zeros_like(bot_cntr_arr)
        bot_cntr_translated[0] = bot_cntr_arr[0] - self.orig_X
        bot_cntr_translated[1] = bot_cntr_arr[1]-self.orig_Y
        #  get robot location relative to env
        bot_on_env = (self.rot_mat @ bot_cntr_translated.T).T
        # e) Translating to get complete Image
        rot_cols = self.orig_rows
        rot_rows = self.orig_cols
        bot_on_env[0] = bot_on_env[0] + (rot_cols * (bot_on_env[0]<0) )  
        bot_on_env[1] = bot_on_env[1] + (rot_rows * (bot_on_env[1]<0) )
        # Update the placeholder for relative location of robot
        self.loc_robot = (int(bot_on_env[0]),int(bot_on_env[1]))

    def localize_bot(self,curr_frame,frame_disp):
        
        # Step 1: Background Model Extraction
        if not self.is_bg_extracted:
            self.extract_bg(curr_frame.copy()) # extract_bg --> main func 
            self.is_bg_extracted = True
            
        # Step 2: Foreground Detection
        change = cv2.absdiff(curr_frame, self.background_model)
        change_gray = cv2.cvtColor(change, cv2.COLOR_BGR2GRAY)
        change_mask = cv2.threshold(change_gray, 15, 255, cv2.THRESH_BINARY)[1]
        robot_mask, rbt_cnt = ret_largest_obj(change_mask) #remove all the noise and leave the robot as largest img

        # Step 3: once we have the robot--> find its location 
        self.get_rbt_loc(rbt_cnt,robot_mask)

        # Drawing bounding circle around detected robot
        center, radii = cv2.minEnclosingCircle(rbt_cnt)
        rbt_circular_mask = cv2.circle(robot_mask.copy(), (int(center[0]), int(center[1])), int(radii+(radii*0.4)), 255, 3)
        rbt_circular_mask = cv2.bitwise_xor(rbt_circular_mask, robot_mask)
        frame_disp[robot_mask>0]  = frame_disp[robot_mask>0] + (0,64,0)
        frame_disp[rbt_circular_mask>0]  = (0,0,255)


        # displaying for GUI
        if (robot_setup.debug and robot_setup.debug_localization):
            cv2.imshow("Background Model",self.background_model)
            cv2.imshow("Environment OG",self.env_og)
            
            cv2.imshow("Robot Found", frame_disp)
            #cv2.imshow("Noise Visible", change_mask) 
            cv2.imshow("Detected Foreground", robot_mask)

        else:
            try:
                cv2.destroyWindow("Background Model")
                cv2.destroyWindow("Environment OG")
                
                cv2.destroyWindow("Robot Found")
                #cv2.destroyWindow("Noise Visible")
                cv2.destroyWindow("Detected Foreground")

                cv2.destroyWindow("Rois Mask")
                cv2.destroyWindow("Robot Frame Removed")
                #cv2.destroyWindow("Ground Replica")

            except:
                pass
