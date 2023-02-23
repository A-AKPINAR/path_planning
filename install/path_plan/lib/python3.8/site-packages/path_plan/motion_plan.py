
import cv2
import numpy as np
from math import pow , atan2,sqrt , degrees,asin

from numpy import interp
import pygame
import os
pygame.mixer.init()
pygame.mixer.music.load(os.path.abspath('src/path_plan/resource/aud_chomp.mp3'))

from . import robot_setup

class motionplanner():


    def __init__(self):

        # counter to move robot forward 
        self.count = 0
        self.pt_i_taken = False
        # container to store Initial robot location
        self.init_loc = 0


        self.angle_relation_computed = False

        # container for robot angle --> type: image 
        self.robot_angle = 0
        # container for robot angle --> Simulation
        self.robot_angle_s = 0
        # container for angle relation btween (Image & Simulation)
        self.robot_angle_rel = 0
        # to check if robot reached to exit
        self.goal_not_reached_flag = True

        self.goal_pose_x = 0 #to store current x and y loc of robot
        self.goal_pose_y = 0
        self.path_iter = 0 #current step of robot along the path 

#variable for obstacle detection
        self.prev_angle_to_turn = 0
        self.Prev_distance_to_goal = 0
        self.prev_path_iter = 0

        self.angle_not_changed = 0 #variables to start obstacle detection if robot is stuck
        self.dist_not_changed = 0
        self.goal_not_changed =0
        self.goal_not_changed_long =0
        self.backpeddling = 0

        # robot stuck on wall --> then  backpeddle
        self.strt_backtrack = False
        #can't reach goal --> try next one
        self.go_nextpnt = False

        self.curr_speed = 0
        self.curr_angle = 0


    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians
  
    def get_pose(self,data):

        quaternions = data.pose.pose.orientation
        (roll,pitch,yaw)=self.euler_from_quaternion(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
        yaw_deg = degrees(yaw)

        if (yaw_deg>0):
            self.robot_angle_s = yaw_deg
        else:
            # -160 + 360 = 200, -180 + 360 = 180 . -90 + 360 = 270
            self.robot_angle_s = yaw_deg + 360

    @staticmethod
    def bck_to_orig(pt,transform_arr,rot_mat):

        st_col = transform_arr[0] # cols X
        st_row = transform_arr[1] # rows Y
        tot_cols = transform_arr[2] # total_cols / width W
        tot_rows = transform_arr[3] # total_rows / height H

        pt_array = np.array( [pt[0], pt[1]] )
        
        # Rot Matrix --> for normal XY convention around z axis = [cos0 -sin0]) But for Image convention [ cos0 sin0]
        #                                                      [sin0  cos0]                           [-sin0 cos0]
        rot_center = (rot_mat @ pt_array.T).T# [x,y]
        
        # Translating Origin -->  (To get whole image)
        rot_cols = tot_cols
        rot_rows = tot_rows
        rot_center[0] = rot_center[0] + (rot_cols * (rot_center[0]<0) ) + st_col  
        rot_center[1] = rot_center[1] + (rot_rows * (rot_center[1]<0) ) + st_row 
        return rot_center

    def display_control_mechanism_in_action(self,bot_loc,path,img_shortest_path,robot_localizer,frame_disp):
        Doing_pt = 0
        Done_pt = 0

        path_i = self.path_iter
        
        # Circle to represent current location of robot
        img_shortest_path = cv2.circle(img_shortest_path, bot_loc, 3, (0,0,255))

        if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
            curr_goal = path[path_i]
            if path_i!=0:
                img_shortest_path = cv2.circle(img_shortest_path, path[path_i-1], 3, (0,255,0),2)
                Done_pt = path[path_i-1]
            img_shortest_path = cv2.circle(img_shortest_path, curr_goal, 3, (0,140,255),2)
            Doing_pt = curr_goal #---> completing first goal
        else:

            img_shortest_path = cv2.circle(img_shortest_path, path[path_i], 10, (0,255,0))
            Done_pt = path[path_i]

        if Doing_pt!=0:
            Doing_pt = self.bck_to_orig(Doing_pt, robot_localizer.transform_arr, robot_localizer.rot_mat_rev)
            frame_disp = cv2.circle(frame_disp, (int(Doing_pt[0]),int(Doing_pt[1])), 3, (0,140,255),2)   

            
        if Done_pt!=0:
            Done_pt = self.bck_to_orig(Done_pt, robot_localizer.transform_arr, robot_localizer.rot_mat_rev)
            if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
                pass 
            else:
                frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 10, (0,255,0))  

        st = "Path Length = ( {} ) , Current Path Step = ( {} )".format(len(path),self.path_iter)        
        
        frame_disp = cv2.putText(frame_disp, st, (robot_localizer.orig_X-50,robot_localizer.orig_Y-30), cv2.FONT_HERSHEY_PLAIN, 1.2, (0,0,255))
        if robot_setup.debug and robot_setup.debug_motionplanning:
            cv2.imshow("Maze: Shortest Path and Robot Location",img_shortest_path)
        else:
            try:
                cv2.destroyWindow("Maze: Shortest Path and Robot Location")
            except:
                pass

    @staticmethod
    def angle_n_dist(pt_a,pt_b):
  
        error_x = pt_b[0] - pt_a[0]
        error_y = pt_a[1] - pt_b[1]

        # calculating distance between two points
        distance = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )

        # calculating angle between two points 
        angle = atan2(error_y,error_x)
        # converting angle from radians to degrees
        angle_deg = degrees(angle)

        if (angle_deg>0):
            return (angle_deg),distance
        else:
    
            return (angle_deg + 360),distance
        
    def check_gtg_status(self,angle_to_turn,distance_to_goal):

 
        change_angle_to_turn = abs(angle_to_turn-self.prev_angle_to_turn)

        if( (abs(angle_to_turn) >5) and (change_angle_to_turn<0.4) and (not self.strt_backtrack) ):
            self.angle_not_changed +=1
            if(self.angle_not_changed>200): #which means robot is stuck on an obstacle 
                self.strt_backtrack = True
        else:
            self.angle_not_changed = 0
        print("[Prev,Change,Not_Changed_Iter,Self.strt_backtrack] = [{:.1f},{:.1f},{},{}] ".format(self.prev_angle_to_turn,change_angle_to_turn,self.angle_not_changed,self.strt_backtrack))
        self.prev_angle_to_turn = angle_to_turn


        change_dist = abs(distance_to_goal-self.Prev_distance_to_goal)

        if( (abs(distance_to_goal) >5) and (change_dist<1.2) and (not self.strt_backtrack) ):
            self.dist_not_changed +=1
            if(self.dist_not_changed>200): #which means robot is stuck on an obstacle 
                self.strt_backtrack = True
        else:
            self.dist_not_changed = 0
        print("[Prev_d,Change_distance,Not_Changed_iter,Self.strt_backtrack] = [{:.1f},{:.1f},{},{}] ".format(self.Prev_distance_to_goal,change_dist,self.dist_not_changed,self.strt_backtrack))
        self.Prev_distance_to_goal = distance_to_goal


        change_goal = self.prev_path_iter - self.path_iter #check if robot goes to next step or not 
        if( (change_goal==0) and (distance_to_goal<30) ):
            self.goal_not_changed +=1
            if(self.goal_not_changed>500):
                self.go_nextpnt = True  #bc next step is not reachable 
        elif(change_goal==0):
            self.goal_not_changed_long+=1
            if(self.goal_not_changed_long>1500):
                self.go_nextpnt = True
        else:
            self.goal_not_changed_long = 0
            self.goal_not_changed = 0
        print("[Prev_g,Change_g,Not_Changed_Iter] = [{:.1f},{:.1f},{}] ".format(self.prev_path_iter,change_goal,self.goal_not_changed))
        self.prev_path_iter = self.path_iter

    @staticmethod
    #func to find next reachable point along the path
    def dist(pt_a,pt_b):
        error_x= pt_b[0] - pt_a[0]
        error_y= pt_a[1] - pt_b[1]
        return( sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) ) )

    def get_suitable_nxtpt(self,rbt_loc,path): 
        extra_i = 1
        test_goal = path[self.path_iter+extra_i]
        
        while(self.dist(rbt_loc, test_goal)<20):
            extra_i+=1
            test_goal = path[self.path_iter+extra_i]
        print("Loading {} pt ".format(extra_i))
        self.path_iter = self.path_iter + extra_i -1


    def go_exit(self,bot_loc,path,velocity,velocity_publisher):
        angle_to_goal,distance_to_goal = self.angle_n_dist(bot_loc, (self.goal_pose_x,self.goal_pose_y)) #second is tuple

        angle_to_turn = angle_to_goal - self.robot_angle #actual angle of robot to move towards exit

        speed = interp(distance_to_goal,[0,100],[0.2,1.5]) #0.2 is the min vel
        self.curr_speed = speed
        angle = interp(angle_to_turn,[-360,360],[-4,4])
        self.curr_angle = angle

        print("Angle to goal = {} Angle to turn = {} Angle in Simulation {}".format(angle_to_goal,angle_to_turn,abs(angle)))
        print("Distance to goal:  = ",distance_to_goal)

        if self.goal_not_reached_flag: #check if robot is too far from the exit 
            self.check_gtg_status(angle_to_turn, distance_to_goal)


        if (distance_to_goal>=3):
            velocity.angular.z = angle
        if abs(angle) < 0.4:
            velocity.linear.x = speed
        elif((abs(angle) < 0.8)):
            velocity.linear.x = 0.02
        else:
            velocity.linear.x = 0.0

        if self.strt_backtrack: #if it is activated --> robot stuck
            print("---> Backtracking (",self.backpeddling,") <---")
            if self.backpeddling==0:
                self.go_nextpnt = True
            velocity.linear.x = -0.16 #move back a nit 
            velocity.angular.z = angle
            self.backpeddling+=1
            # stop backpeddling after some time
            if self.backpeddling == 100:
                self.strt_backtrack = False
                self.backpeddling = 0
                print("---> Backtracking is DONE <---")
        
        #  publishing the updated velocity until exit not reached
        if (self.goal_not_reached_flag) or (distance_to_goal<=1):
            velocity_publisher.publish(velocity)


        if ((distance_to_goal<=8) or self.go_nextpnt):
            if self.go_nextpnt:
                if self.backpeddling:
                    self.get_suitable_nxtpt(bot_loc,path) #find the next feasable one 
                self.go_nextpnt = False
                

            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            if self.goal_not_reached_flag:
                velocity_publisher.publish(velocity)

            # check if the robot reached the exit
            if self.path_iter==(len(path)-1):
                if self.goal_not_reached_flag:
                    self.goal_not_reached_flag = False #confirm we reached the exit!
                    pygame.mixer.music.load(os.path.abspath('src/path_plan/resource/Goal_reached.wav'))
                    pygame.mixer.music.play()
            # if still not reached the exit:
            else:
                # Iterate over
                self.path_iter += 1
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]
                
                if pygame.mixer.music.get_busy() == False:
                    pygame.mixer.music.play()

#main func
    def nav_path(self,bot_loc,path,velocity,velocity_publisher):

        # If valid path is found
        if (type(path)!=int):
            if (self.path_iter==0): #means no step is taken within the path 
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]

        if (self.count >20):

            if not self.angle_relation_computed:

                velocity.linear.x = 0.0
                # Stopping the robot
                velocity_publisher.publish(velocity)
                self.robot_angle, _= self.angle_n_dist(self.init_loc, bot_loc)  #compute abgle of robot in img  --> we want first output which is robot angle 
                self.robot_angle_init = self.robot_angle
                self.robot_angle_rel = self.robot_angle_s - self.robot_angle #compute robot angle relation
                self.angle_relation_computed = True

            else:
                self.robot_angle = self.robot_angle_s - self.robot_angle_rel #complete pose is computed

                print("\n\nRobot Angle in Image = {} I-S Relation {} Robot angle in Simulation = {}".format(self.robot_angle,self.robot_angle_rel,self.robot_angle_s))
                print("Robot Angle Initial (Image) = ",self.robot_angle_init)
                print("Robot location {}".format(bot_loc))

                
                self.go_exit(bot_loc,path,velocity,velocity_publisher) #simply telling robot to go to next step until the its's an exit


        else:
            # If bot initial location not already taken
            if not self.pt_i_taken:
                # Set init_loc = Current bot location
                self.init_loc = bot_loc
                self.pt_i_taken = True
                
            # Keep moving forward for 20 iterations(count)
            velocity.linear.x = 1.0
            velocity_publisher.publish(velocity)

            self.count+=1