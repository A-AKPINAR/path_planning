''' Purpose of this script to detect available nodes / paths and connect them to have a path to follow for robot navigation'''

import cv2
import numpy as np
from . import robot_setup

draw_intrstpts = True
debug_mapping = False

class Graph():

    def __init__(self):
     
        self.graph = {}
        self.start = 0
        self.end = 0


    #Func is to add new vertex to graph and connect w its neighbor if there is any vertex is detected
    def add_vertex(self,vertex,neighbor= None,case = None, cost = None):
        
        # If we find a vertex then create connection 
        if vertex in self.graph.keys():
            self.graph[vertex][neighbor] = {}
            self.graph[vertex][neighbor]["case"] = case
            self.graph[vertex][neighbor]["cost"] = cost
        else:
            # If there is none --> add new vertex to graph
            self.graph[vertex] = {}
            self.graph[vertex]["case"] = case

    # Function to display complete graph
    def displaygraph(self):
        for key,value in self.graph.items():
            print("Key {} has value {} ".format(key,value))

class mapper():

    def __init__(self):

        self.graphified = False

        # cropping control -->  removing maze boundary
        self.crp_amt = 5

        # graph object for storing env
        self.Graph = Graph()


        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False

        # Colored env for displaying connection between nodes
        self.robot_cnnct = []

        
        self.maze = 0

    # Display connection between nodes with a colored line
    def display_connected_nodes(self,curr_node,neighbor_node,case="Unkown",color=(0,0,255)):
        curr_pixel = (curr_node[1],curr_node[0])
        neighbor_pixel = (neighbor_node[1],neighbor_node[0])

        print("CONNECTED -------->> {} << ".format(case))
        self.robot_cnnct = cv2.line(self.robot_cnnct,curr_pixel,neighbor_pixel,color,1)
        if robot_setup.debug and robot_setup.debug_mapping:
             cv2.imshow("Nodes Conected", self.robot_cnnct)
        if debug_mapping:
             cv2.waitKey(0)                    
             self.robot_cnnct = cv2.line(self.robot_cnnct,curr_pixel,neighbor_pixel,(255,255,255),1)

    # Connect curr_node to its neighbors 
    def connect_neighbors(self,maze,node_row,node_col,case,step_l = 1,step_up = 0,totl_cnncted = 0):
        
        curr_node = (node_row,node_col)

        # check if there is a path around  node        
        if (maze[node_row-step_up][node_col-step_l]>0):
            # If there is a path --> look for neighbor node to connect
            neighbor_node = (node_row-step_up,node_col-step_l)

            # if neighbor node is already in graph:               
            if neighbor_node in self.Graph.graph.keys():
                neighbor_case = self.Graph.graph[neighbor_node]["case"]
                cost = max(abs(step_l),abs(step_up))
                totl_cnncted +=1

                self.Graph.add_vertex(curr_node,neighbor_node,neighbor_case,cost)
                self.Graph.add_vertex(neighbor_node,curr_node,case,cost)
                print("\nConnected {} to {} with  = [ {} , {} ] & cost -> {}".format(curr_node,neighbor_node,step_l,step_up,cost))

              
                if not self.connected_left:
                    self.display_connected_nodes(curr_node, neighbor_node,"LEFT",(0,0,255))
                    # Vertex --> left neighbor.                    
                    self.connected_left = True
                    # Check left corner 
                    step_l = 1
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_upleft:
                    self.display_connected_nodes(curr_node, neighbor_node,"UPLEFT",(0,128,255))
                    # Vertex -->  up-left neighbor.
                    self.connected_upleft = True
                    # top route 
                    step_l  = 0
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_up:
                    self.display_connected_nodes(curr_node, neighbor_node,"UP",(0,255,0))
                    # Vertex to --> its up neighbor.
                    self.connected_up = True
                    # top-right route 
                    step_l  = -1
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_upright:
                    self.display_connected_nodes(curr_node, neighbor_node,"UPRIGHT",(255,0,0))
                    # Vertex --> its up-right neighbor.
                    self.connected_upright = True

            
            if not self.connected_upright:
                if not self.connected_left:
                    step_l +=1
                elif not self.connected_upleft:
                    step_l+=1
                    step_up+=1
                elif not self.connected_up:
                    step_up+=1
                elif not self.connected_upright:           
                    step_l-=1
                    step_up+=1
                self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
        else:
          # No path in the current direction --> next direction
            if not self.connected_left:
               
                self.connected_left = True
                # Looking upleft now
                step_l = 1
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)

            elif not self.connected_upleft:
               
                self.connected_upleft = True
                step_l = 0
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, totl_cnncted)
                

            elif not self.connected_up:
               
                self.connected_up = True
                step_l = -1
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, totl_cnncted)

            elif not self.connected_upright:
          
                self.connected_upright = True
                step_l = 0
                step_up = 0                
                return     
    
    # function to draw a triangle around a point    
    @staticmethod
    def triangle(image,ctr_pt,radius,colour=(0,255,255),thickness=1):
      
        pts = np.array( [ [ctr_pt[0]        , ctr_pt[1]-radius]  , 
                          [ctr_pt[0]-radius , ctr_pt[1]+radius]  ,
                          [ctr_pt[0]+radius , ctr_pt[1]+radius]   
                        ] 
                        ,np.int32
                      )
        
        pts = pts.reshape((-1, 1, 2))
        
        image = cv2.polylines(image,[pts],True,colour,thickness)
        return image
    
    # function to get surrounding pixels intensities for any point
    @staticmethod
    def get_surround_pixel_intensities(maze,curr_row,curr_col):

        # binary thrsholding  (+ values ==> 1 n - values ==> 0)
        maze = cv2.threshold(maze, 1, 1, cv2.THRESH_BINARY)[1]

        rows = maze.shape[0]
        cols = maze.shape[1]

        # boundary condition
        top_row = False
        btm_row = False
        lft_col = False
        rgt_col = False

        # Checking if there is a boundary condition
        if (curr_row==0):
            top_row = True
        if (curr_row == (rows-1)):
            btm_row = True
        if (curr_col == 0):
            lft_col = True
        if (curr_col == (cols-1)):
            rgt_col = True

        # extracting surround pixel intensities and addressing boundary conditions 
        if (top_row or lft_col):
            top_left = 0
        else:
            top_left = maze[curr_row-1][curr_col-1]
        if( top_row or rgt_col ):
            top_rgt = 0
        else:
            top_rgt = maze[curr_row-1][curr_col+1]

        if( btm_row or lft_col ):
            btm_left = 0
        else:
            btm_left = maze[curr_row+1][curr_col-1]

        if( btm_row or rgt_col ):
            btm_rgt = 0
        else:
            btm_rgt = maze[curr_row+1][curr_col+1]
        

        if (top_row):
            top = 0
        else:
            top = maze[curr_row-1][curr_col]
        if (rgt_col):
            rgt = 0
        else:
            rgt = maze[curr_row][curr_col+1]
        
        if (btm_row):
            btm = 0
        else:
            btm = maze[curr_row+1][curr_col]

        if (lft_col):
            lft = 0
        else:
            lft = maze[curr_row][curr_col-1]

        no_of_pathways = ( top_left + top      + top_rgt  +
                           lft      + 0        + rgt      + 
                           btm_left + btm      + btm_rgt        
                         )
        if no_of_pathways>2:  
            print("  [ Top_Left , Top      , Top_Right  ,Left    , Right      , Bottm_Left , Bottom      , Bottom_Right   ] \n [ ",str(top_left)," , ",str(top)," , ",str(top_rgt)," ,\n   ",str(lft)," , ","-"," , ",str(rgt)," ,\n   ",str(btm_left)," , ",str(btm)," , ",str(btm_rgt)," ] ")
            print("\nNumber_of_Pathways [row,col]= [ ",curr_row," , ",curr_col," ] ",no_of_pathways) 

        return top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft,no_of_pathways
    # Reset state parameters of each vertex connection
    def reset_connct_paramtrs(self):
        # Reseting member variables to False initially when looking for nodes to connect
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False

    def one_pass(self,maze):

        # remove previously found nodes
        self.Graph.graph.clear()

        self.robot_cnnct = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR) #reset the map


       
        turns = 0
        junc_3 = 0
        junc_4 = 0

        # Converting env to Colored --> for IP
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        # Creating a window to display detected Nodes
        rows = maze.shape[0]
        cols = maze.shape[1]

        # Looping over each pixel from left to right --> bottom to top
        for row in range(rows):
            for col in range(cols):

                if (maze[row][col]==255):
                    if debug_mapping:
                       
                        self.robot_cnnct = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
                  
                    top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft, paths = self.get_surround_pixel_intensities(maze.copy(),row,col)

                    if ( (row==0) or (row == (rows-1)) or (col==0) or (col == (cols-1)) ):
                        if (row == 0):
                            # Start
                            maze_bgr[row][col] = (0,128,255)
                            if robot_setup.debug and robot_setup.debug_mapping:
                                 cv2.imshow("Environment with Interest Points",maze_bgr)
                             # Adding to graph
                            self.Graph.add_vertex((row,col),case="Start ")
                            self.Graph.start = (row,col)


                        else:
                            # exit point
                            maze_bgr[row][col] = (0,255,0)
                            if robot_setup.debug and robot_setup.debug_mapping:
                                 cv2.imshow("Environment with Interest Points",maze_bgr)
                             # Add found vertex to graph n env exit to graph-end
                            self.Graph.add_vertex((row,col),case="End ")
                            self.Graph.end = (row,col)
                             # if there is any then --> connect vertex to its neighbor
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "End ")

                    # checking dead end -- >red color w circle shape
                    elif (paths==1):
                        crop = maze[row-1:row+2,col-1:col+2]
                        print(" ** Dead End ** \n" ,crop)
                        maze_bgr[row][col] = (0,0,255)
                        if draw_intrstpts:
                            maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (0,0,255),2)
                        if robot_setup.debug and robot_setup.debug_mapping:
                             cv2.imshow("Environment with Interest Points",maze_bgr)
                         # add found dead end to graph
                        self.Graph.add_vertex((row,col),case = "DeadEnd ")
                        self.reset_connct_paramtrs()
                        self.connect_neighbors(maze, row, col, "DeadEnd ")

                    # Check if it is either a turn or a normal path
                    elif (paths==2):
                        crop = maze[row-1:row+2,col-1:col+2]
                        nzero_loc = np.nonzero(crop > 0)
                        nzero_ptA = (nzero_loc[0][0],nzero_loc[1][0])
                        nzero_ptB = (nzero_loc[0][2],nzero_loc[1][2])
                        if not ( ( (2 - nzero_ptA[0])==nzero_ptB[0] ) and 
                                    ( (2 - nzero_ptA[1])==nzero_ptB[1] )     ):
                            if robot_setup.debug and robot_setup.debug_mapping:
                                 cv2.imshow("Environment with Interest Points",maze_bgr)
                            self.Graph.add_vertex((row,col),case = "Turn ")
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "Turn ")
                            turns+=1
                    #  *3-Junc* or a *4-Junc*
                    # 3 junction --> triangle n lime color
                    #4 junction --> rectangle n orange color
                    elif (paths>2):
                        if (paths ==3):
                            maze_bgr[row][col] = (0,255,0)
                            if draw_intrstpts:
                                maze_bgr = self.triangle(maze_bgr, (col,row), 10,(144,140,255))
                            if robot_setup.debug and robot_setup.debug_mapping:
                                 cv2.imshow("Environment with Interest Points",maze_bgr)
                             # add found vertex to graph
                            self.Graph.add_vertex((row,col),case = "3-Junction ")
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "3-Junction")
                            junc_3+=1                                      
                        else:
                            maze_bgr[row][col] = (255,165,0)
                            if draw_intrstpts:
                                cv2.rectangle(maze_bgr,(col-10,row-10) , (col+10,row+10), (255,140,144),2)
                            if robot_setup.debug and robot_setup.debug_mapping:
                                 cv2.imshow("Environment with Interest Points",maze_bgr)
                             # add found vertex to graph
                            self.Graph.add_vertex((row,col),case = "4-Junction ")
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "4-Junction ")
                            junc_4+=1
        self.robot_cnnct = maze_bgr

        print("Found Node Points !!! \n[ Turns , 3_Junc , 4_Junc ] [ ",turns," , ",junc_3," , ",junc_4," ] \n")


    #Here we convert env to a graph
    def graphify(self,extracted_maze):

        # Check graph extracted or not
        if not self.graphified:

            # Step 1:  thinning on maze to reduce area to paths that rbot could follow.
            thinned = cv2.ximgproc.thinning(extracted_maze)
            # Step 2: Dilate and Perform thining again to minimize unneccesary interest point (i.e:turns)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
            thinned_dilated = cv2.morphologyEx(thinned, cv2.MORPH_DILATE, kernel)
            _, bw2 = cv2.threshold(thinned_dilated, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)        
            thinned = cv2.ximgproc.thinning(bw2)
            # Step 3: Crop out Boundary that is not part
            thinned_cropped = thinned[self.crp_amt:thinned.shape[0]-self.crp_amt,
                                      self.crp_amt:thinned.shape[1]-self.crp_amt]
            # Step 4: Overlay found path on Maze Occupency Grid.
            extracted_maze_cropped = extracted_maze[self.crp_amt:extracted_maze.shape[0]-self.crp_amt,
                                                    self.crp_amt:extracted_maze.shape[1]-self.crp_amt]
            extracted_maze_cropped = cv2.cvtColor(extracted_maze_cropped, cv2.COLOR_GRAY2BGR)
            extracted_maze_cropped[thinned_cropped>0] = (255,0,0)

            # Step 5: Identify Nodes in the path to further reduce processing time
            self.one_pass(thinned_cropped)
            self.maze = thinned_cropped
            self.graphified = True

            if robot_setup.debug and robot_setup.debug_mapping:
                cv2.imshow("Extracted Environment",extracted_maze)
                cv2.imshow('Environment (thinned*2)(Cropped)', thinned_cropped)
        else:

            if robot_setup.debug and robot_setup.debug_mapping:
                #cv2.imshow("Nodes Conected", self.robot_cnnct)
                cv2.imshow("Environment with Interest Points", self.robot_cnnct)
            else:
                try:
                    cv2.destroyWindow("Nodes Conected")
                    cv2.destroyWindow("Environment with Interest Points")
                    cv2.destroyWindow("Extracted Environment")
                    cv2.destroyWindow('Environment(thinned)')
                    #cv2.destroyWindow('Envrionment (thinned*2)')

                except:
                    pass






