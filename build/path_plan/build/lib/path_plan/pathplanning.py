
import cv2
import numpy as np
from numpy import sqrt

from . import robot_setup
class pathplan():

    def __init__(self):
        self.DFS = DFS()
        self.dijisktra = dijisktra()
        self.astar = a_star()
        #self.rrt = Node()

        self.path_to_goal = []
        self.img_shortest_path = []
        self.choosen_route = []
        

    @staticmethod
    def cords_to_pts(cords):
      return [cord[::-1] for cord in cords]

    def draw_path_on_maze(self,maze,shortest_path_pts,method):
        
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        self.choosen_route = np.zeros_like(maze_bgr)

        rang = list(range(0,254,25))
        
        depth = maze.shape[0]
        for i in range(len(shortest_path_pts)-1):
            per_depth = (shortest_path_pts[i][1])/depth

            color = ( 
                      int(255 * (abs(per_depth+(-1*(per_depth>0.5)))*2) ),
                      int(255 * per_depth),
                      int(255 * (1-per_depth))
                    )
            cv2.line(maze_bgr,shortest_path_pts[i] , shortest_path_pts[i+1], color)
            cv2.line(self.choosen_route,shortest_path_pts[i] , shortest_path_pts[i+1], color,3)

        img_str = "Maze w Found Path [" +method +"]"
        if robot_setup.debug and robot_setup.debug_pathplanning:
            cv2.namedWindow(img_str,cv2.WINDOW_FREERATIO)
            cv2.imshow(img_str, maze_bgr)

        if method == "dijisktra":
            self.dijisktra.shortest_path_overlayed = maze_bgr
        elif method == "a_star":
            self.astar.shortest_path_overlayed = maze_bgr
            
        self.img_shortest_path = maze_bgr.copy()

    def find_path_nd_display(self,graph,start,end,maze,method = "DFS"):

        Path_str = "Path"
        
        if method=="DFS":
            paths = self.DFS.get_paths(graph, start, end)
            path_to_display = paths[0]

        elif (method == "DFS_Shortest"):
            paths_N_costs = self.DFS.get_paths_cost(graph,start,end)
            paths = paths_N_costs[0]
            costs = paths_N_costs[1]
            min_cost = min(costs)
            path_to_display = paths[costs.index(min_cost)]
            Path_str = "Shortest "+ Path_str

        elif (method == "dijisktra"):
            
            if not self.dijisktra.shortestpath_found:
                print("Finding Shortest Routes")
                self.dijisktra.find_best_routes(graph, start, end)
            
            path_to_display = self.dijisktra.shortest_path
            Path_str = "Shortest "+ Path_str

        elif (method == "a_star"):
            
            if not self.astar.shortestpath_found:
                print("Finding Shortest Routes")
                self.astar.find_best_routes(graph, start, end)
            
            path_to_display = self.astar.shortest_path
            Path_str = "\nShortest "+ Path_str


        skip_s_goals = 1
        path_to_display_rdcd = path_to_display[skip_s_goals:len(path_to_display)]
        pathpts_to_display = self.cords_to_pts(path_to_display_rdcd)

        self.path_to_goal = pathpts_to_display

        
        if robot_setup.debug and robot_setup.debug_pathplanning:
            print(Path_str," from {} to {} is =  {}".format(start,end,pathpts_to_display))

        if (method =="dijisktra"):
            if (self.dijisktra.shortest_path_overlayed == []):
                self.draw_path_on_maze(maze,pathpts_to_display,method)
            else:
                if robot_setup.debug and robot_setup.debug_pathplanning:
                    cv2.imshow("Maze w Found Path [Dijisktra]", self.dijisktra.shortest_path_overlayed)
                else:
                    try:
                        cv2.destroyWindow("Maze w Found Path [Dijisktra]")
                    except:
                        pass

        elif (method == "a_star"):
            if (self.astar.shortest_path_overlayed == []):
                self.draw_path_on_maze(maze,pathpts_to_display,method)
            else:
                if robot_setup.debug and robot_setup.debug_pathplanning:
                    cv2.imshow("Maze w Found Path [A*]", self.astar.shortest_path_overlayed)
                else:
                    try:
                        cv2.destroyWindow("Maze w Found Path [A*]")
                    except:
                        pass
                

        


class DFS():


    @staticmethod
    def get_paths(graph,start,end,path = []):
        
        # Update the path to where you been already 
        path = path + [start]

        # 2) Define the simplest case
        if (start == end):
            return [path]


        if start not in graph.keys():
            return []
        # List to store all possible paths from start to end
        paths = []

        for node in graph[start].keys():
  
            if ( (node not in path) and (node!="case") ):
                new_paths = DFS.get_paths(graph,node,end,path)
                for p in new_paths:
                    paths.append(p)

        return paths

    
    # Retrieve all possible paths and their costs/time to reaching the goal node
    @staticmethod
    def get_paths_cost(graph,start,end,path=[],cost=0,trav_cost=0):

        # Update the path and the cost to reaching that path
        path = path + [start]
        cost = cost + trav_cost


        if start == end:
            return [path],[cost]
        if start not in graph.keys():
            return [],0

        # List to store all possible paths from point A to B
        paths = []
        # List to store costs of each possible path to goal
        costs = []


        for node in graph[start].keys():

            if ( (node not in path) and (node!="case") ):

                new_paths,new_costs = DFS.get_paths_cost(graph,node, end,path,cost,graph[start][node]['cost'])

                for p in new_paths:
                    paths.append(p)
                for c in new_costs:
                    costs.append(c)
        
        return paths,costs


# Heap class to be used as a priority queue for dijisktra and A*
class Heap():

    def __init__(self):
        # Priority queue will be stored in an array --> list of list containing vertex and their resp distance
        self.array = []
        self.size = 0
        # Curr_pos of each vertex is stored
        self.posOfVertices = []

    # create a minheap node --> type: List(vertex,distance)
    def new_minHeap_node(self,v,dist):
        return([v,dist])

    def swap_nodes(self,a,b):
        temp = self.array[a]
        self.array[a] = self.array[b]
        self.array[b] = temp

    def minHeapify(self,node_idx):
        smallest = node_idx
        left = (node_idx*2)+1
        right = (node_idx*2)+2

        if ((left<self.size) and (self.array[left][1]<self.array[smallest][1])):
            smallest = left
        if ((right<self.size) and (self.array[right][1]<self.array[smallest][1])):
            smallest = right

        if(smallest != node_idx):
            # Update the positions to keep smallest on top
            self.posOfVertices[self.array[node_idx][0]] = smallest
            self.posOfVertices[self.array[smallest][0]] = node_idx
            self.swap_nodes(node_idx, smallest)
 
            self.minHeapify(smallest)


    def extractmin(self):

        # Handling boudary condtion
        if self.size == 0:
            return

        root = self.array[0]
        lastNode = self.array[self.size-1]
        self.array[0] = lastNode

        # Update the postion of vertices
        self.posOfVertices[root[0]] = self.size-1
        self.posOfVertices[lastNode[0]] = 0

        # Decrease the size of minheap by 1
        self.size-=1
        self.minHeapify(0)
        return root

    # Update distance for a node to a new found shorter distance
    def decreaseKey(self,vertx,dist):
        
        idxofvertex = self.posOfVertices[vertx]

        self.array[idxofvertex][1] = dist

        while((idxofvertex>0) and (self.array[idxofvertex][1]<self.array[(idxofvertex-1)//2][1])):
            # Update position of parent and curr_node
            self.posOfVertices[self.array[idxofvertex][0]] = (idxofvertex-1)//2 
            self.posOfVertices[self.array[(idxofvertex-1)//2][0]] = idxofvertex
            self.swap_nodes(idxofvertex, (idxofvertex-1)//2)
            idxofvertex = (idxofvertex-1)//2

    #  function to check if a given vertex is already in min heap or not 

    def isInMinHeap(self, v):
 
        if self.posOfVertices[v] < self.size:
            return True
        return False

class dijisktra():

    def __init__(self):
        
        # State variable 
        self.shortestpath_found = False
        # Once found save the shortest path
        self.shortest_path = []

        self.shortest_path_overlayed = []
        self.minHeap = Heap()
        
        # Creating dictionaries to manage the world
        self.idxs2vrtxs = {}
        self.vrtxs2idxs = {}
        self.dijiktra_nodes_visited = 0

    def ret_shortestroute(self,parent,start,end,route):

        route.append(self.idxs2vrtxs[end])

        if (end==start): #point we the shortest path is found 
            return
        
        # Visit closest vertex to each node
        end = parent[end]
        # Recursively call function with new end point until reaching start
        self.ret_shortestroute(parent, start, end, route)

    def find_best_routes(self,graph,start,end):

        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0]==start][0]
        print("Index of search key : {}".format(start_idx))

        # Distanc list storing dist of each node
        dist = []       
        parent = []

        # Set size of minHeap to be the total no of keys in the graph.
        self.minHeap.size = len(graph.keys())

        for idx,v in enumerate(graph.keys()):

            dist.append(1e7)
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx]))
            self.minHeap.posOfVertices.append(idx)
            parent.append(-1)
            # Updating dictionaries of vertices and their positions
            self.vrtxs2idxs[v] = idx
            self.idxs2vrtxs[idx] = v

    
        dist[start_idx] = 0
        self.minHeap.decreaseKey(start_idx, dist[start_idx])


        while(self.minHeap.size!=0):
            

            self.dijiktra_nodes_visited += 1
            curr_top = self.minHeap.extractmin()
            u_idx = curr_top[0]
            u = self.idxs2vrtxs[u_idx]

            # check all neighbors of vertex u and update their distance if found shorter
            for v in graph[u]:
                if v!= "case":
                    print("Vertex adjacent to {} is {}".format(u,v))
                    v_idx = self.vrtxs2idxs[v]

                    #if shortest distance is not found --> to v + new found dist < known dist ==> Update dist for v
                    if ( self.minHeap.isInMinHeap(v_idx) and (dist[u_idx]!=1e7) and
                       (    (graph[u][v]["cost"] + dist[u_idx]) < dist[v_idx] )    ):

                       dist[v_idx] = graph[u][v]["cost"] + dist[u_idx]
                       self.minHeap.decreaseKey(v_idx, dist[v_idx])
                       parent[v_idx] = u_idx
            
            # end condition: End goal has already been visited. 
            if (u == end):
                break
        
        shortest_path = []
        self.ret_shortestroute(parent, start_idx,self.vrtxs2idxs[end],shortest_path)
        
        self.shortest_path = shortest_path[::-1]
        self.shortestpath_found = True


class a_star(dijisktra):

    def __init__(self):

        super().__init__()
        self.astar_nodes_visited = 0

    @staticmethod
    def euc_d(a,b):
        return sqrt( pow( (a[0]-b[0]),2 ) + pow( (a[1]-b[1]),2 ) )


    # Function Ovverrriding
    def find_best_routes(self,graph,start,end):

        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0]==start][0]
        print("Index of search key : {}".format(start_idx))

        # Cost of reaching that node from start
        cost2node = []
        # Distanc list storing dist of each node
        dist = []       
        parent = []

        # Set size of minHeap to be the total no of keys in the graph.
        self.minHeap.size = len(graph.keys())

        for idx,v in enumerate(graph.keys()):


            cost2node.append(1e7)
            dist.append(1e7)
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx]))
            self.minHeap.posOfVertices.append(idx)
            parent.append(-1)
            self.vrtxs2idxs[v] = idx
            self.idxs2vrtxs[idx] = v

        cost2node[start_idx] = 0
        # Total cost(Start Node) = Cost2Node(Start) + Heuristic Cost(Start,End)
        dist[start_idx] = cost2node[start_idx] + self.euc_d(start, end)
        self.minHeap.decreaseKey(start_idx, dist[start_idx])


        while(self.minHeap.size!=0):
            self.astar_nodes_visited += 1

            curr_top = self.minHeap.extractmin()
            u_idx = curr_top[0]
            u = self.idxs2vrtxs[u_idx]

            # check all neighbors of vertex u and update their distance if found shorter
            for v in graph[u]:
                if v!= "case":

                    print("Vertex adjacent to {} is {}".format(u,v))
                    v_idx = self.vrtxs2idxs[v]

        
                    if ( self.minHeap.isInMinHeap(v_idx) and (dist[u_idx]!=1e7) and
                       (    (graph[u][v]["cost"] + cost2node[u_idx]) < cost2node[v_idx] )    ):

                       cost2node[v_idx] = graph[u][v]["cost"] + cost2node[u_idx]
                       dist[v_idx] = cost2node[v_idx] + self.euc_d(v, end)
                       self.minHeap.decreaseKey(v_idx, dist[v_idx])
                       parent[v_idx] = u_idx
            
     

            if (u == end):
                break
        
        shortest_path = []
        self.ret_shortestroute(parent, start_idx,self.vrtxs2idxs[end],shortest_path)
        
        # Return route (reversed) to start from the beginning
        self.shortest_path = shortest_path[::-1]
        self.shortestpath_found = True


