import numpy as np
import time
import cv2 as cv
import heapq

class Map:
    
    def __init__(self):
        # Visualizing the path by writing video
        self.height=500
        self.width=1200
        frameSize = (self.width, self.height)
        fourcc = cv.VideoWriter_fourcc('m','p','4','v')
        self.out = cv.VideoWriter('djkstra_rohit_suresh.mp4', fourcc, 60, frameSize)
        self.map=None
        self.clearance=5

        
    # Function to create the map and obstacle map
    def create_map(self):
        # Main map for display
        map=np.ones((self.height,self.width,3),dtype=np.uint8)
        map[:,:,0]=233
        map[:,:,1]=230
        map[:,:,2]=221

        # Obstacle map for path planning
        obs_map=np.ones((self.height,self.width),dtype=np.uint8)*255

        # Draw obstacles on the maps with clearance
        for i in range(self.width):
            for j in range(self.height):
                # Walls
                if i<self.clearance or i>1200-self.clearance or j<self.clearance or j>500-self.clearance:
                    map[j][i]=(2, 116, 189)
                    obs_map[j][i]=0
                # Rectangles
                if (i>=100-self.clearance and i<=175+self.clearance and j<=400) or (i>=275-self.clearance and i<=350+self.clearance and j>=100):
                    map[j][i]=(2, 116, 189)
                    obs_map[j][i]=0
                # Hexagon
                if ((150+self.clearance)/2)*abs(i-650)/(150+self.clearance)+100<=j<=300-((150+self.clearance)/2)*abs(i-650)/(150+self.clearance)+100 and 510-self.clearance<=i<=790+self.clearance:
                    map[j][i]=(2, 116, 189)
                    obs_map[j][i]=0
                # C- shaped Obstacle
                if (i>=900-self.clearance and i<=1100+self.clearance and j>=50-self.clearance and j<=125+self.clearance) or (i>=900-self.clearance and i<=1100+self.clearance and j>=375-self.clearance and j<=450+self.clearance) or (i>=1020-self.clearance and i<=1100+self.clearance and j>=50-self.clearance and j<=450+self.clearance):
                    map[j][i]=(2, 116, 189)
                    obs_map[j][i]=0
                
        # Draw obstacles on the maps          
        for i in range(self.width):
            for j in range(self.height):
                # Rectangles
                if (i>=100 and i<=175 and self.clearance<=j<=400-self.clearance) or (i>=275 and i<=350 and 500-self.clearance>=j>=100+self.clearance):
                    map[j][i]=(245, 114, 81)
                    obs_map[j][i]=0
                # Hexagon
                if (150/2)*abs(i-650)/150+105<=j<=300-(150/2)*abs(i-650)/150+95 and 510<=i<=790:
                    map[j][i]=(245, 114, 81)
                    obs_map[j][i]=0
                # C- shaped Obstacle
                if (i>=900 and i<=1100 and j>=50 and j<=125) or (i>=900 and i<=1100 and j>=375 and j<=450) or (i>=1020 and i<=1100 and j>=50 and j<=450):
                    map[j][i]=(245, 114, 81)
                    obs_map[j][i]=0

        
        return map,obs_map

    # Saving the map
    def generate_path_map(self):
        print("\nGenerating the map with path:")
        for c,i in enumerate(self.nodes.keys()):
            if self.nodes[i][1]!=float('inf'):
                # Explored node
                cv.circle(self.map,(i[0],i[1]),1,(234,143,234),-1)
                # Start node
                cv.circle(self.map,(self.start_node[0],self.start_node[1]),1,(0,0,255),-1)
                # End node
                cv.circle(self.map,(self.end_node[0],self.end_node[1]),1,(0,255,0),-1)
                if c % 1000==0:
                    self.out.write(self.map)
        for i in range(len(path)):
            cv.circle(self.map,(path[i][0],path[i][1]),1,(196, 173, 157),-1)
            self.out.write(self.map)
        for i in range(300):
            self.out.write(self.map)
        print("Map saved as djkstra_rohith.mp4\n")
        self.out.release()

class Node:
    
    def __init__(self):
        # Dictionary to store the nodes and their costs
        self.nodes={}                        
        
        # Start and end nodes(Initialised to None)
        self.start_node=None            
        self.end_node=None                   

    # Function to insert a node into the nodes dictionary
    def insert_node(self,cost=None,node=None,parent=None):    
        if len(self.nodes)==0:
            # Inserting obstacle nodes into the nodes dictionary
            for i in range(self.width):
                for j in range(self.height):
                    if self.obs_map[j][i]==0:
                        self.nodes.update({(i,j):[None,float('inf')]})
        else:
            self.nodes.update({node:[parent,cost]})
            
    # Action functions to move the point robot in 8 directions
    def Actions(self,node):
        
        # Function to move the point robot to the left
        def MoveLeft_Action(node):
            i,j=node
            if i==0:
                return None,None
            else:
                i=i-1
                j=j
                cost=1
                if (i,j) in self.nodes.keys():
                    if self.nodes[(i,j)][1]==float('inf'):
                        return None,None
                    else:
                        return (i,j),cost
                else:
                    return (i,j),cost

        # Function to move the point robot to the right
        def MoveRight_Action(node):
            i,j=node
            if i==self.width:
                return None,None
            else:
                i=i+1
                j=j
                cost=1
                if (i,j) in self.nodes.keys():
                    if self.nodes[(i,j)][1]==float('inf'):
                        return None,None
                    else:
                        return (i,j),cost
                else:
                    return (i,j),cost

        # Function to move the point robot up
        def MoveUp_Action(node):
            i,j=node
            if j==0:
                return None,None
            else:
                i=i
                j=j-1
                cost=1
                if (i,j) in self.nodes.keys():
                    if self.nodes[(i,j)][1]==float('inf'):
                        return None,None
                    else:
                        return (i,j),cost
                else:
                    return (i,j),cost

        # Function to move the point robot down
        def MoveDown_Action(node):
            i,j=node
            if j==self.height:
                return None,None
            else:
                i=i
                j=j+1
                cost=1
                if (i,j) in self.nodes.keys():
                    if self.nodes[(i,j)][1]==float('inf'):
                        return None,None
                    else:
                        return (i,j),cost
                else:
                    return (i,j),cost

        # Function to move the point robot diagonally up left
        def MoveUpLeft_Action(node):
            i,j=node
            if i==0 or j==0:
                return None,None
            else:
                i=i-1
                j=j-1
                cost=1.4
                if (i,j) in self.nodes.keys():
                    if self.nodes[(i,j)][1]==float('inf'):
                        return None,None
                    else:
                        return (i,j),cost
                else:
                    return (i,j),cost

        # Function to move the point robot diagonally up right
        def MoveUpRight_Action(node):
            i,j=node
            if i==self.width or j==0:
                return None,None
            else:
                i=i+1
                j=j-1
                cost=1.4
                if (i,j) in self.nodes.keys():
                    if self.nodes[(i,j)][1]==float('inf'):
                        return None,None
                    else:
                        return (i,j),cost
                else:
                    return (i,j),cost

        # Function to move the point robot diagonally down left
        def MoveDownLeft_Action(node):
            i,j=node
            if i==0 or j==self.height:
                return None,None
            else:
                i=i-1
                j=j+1
                cost=1.4
                if (i,j) in self.nodes.keys():
                    if self.nodes[(i,j)][1]==float('inf'):
                        return None,None
                    else:
                        return (i,j),cost
                else:
                    return (i,j),cost

        # Function to move the point robot diagonally down right
        def MoveDownRight_Action(node):
            i,j=node
            if i==self.width or j==self.height:
                return None,None
            else:
                i=i+1
                j=j+1
                cost=1.4
                if (i,j) in self.nodes.keys():
                    if self.nodes[(i,j)][1]==float('inf'):
                        return None,None
                    else:
                        return (i,j),cost
                else:
                    return (i,j),cost
            
        return [(MoveLeft_Action(node)), (MoveUp_Action(node)), (MoveRight_Action(node)), (MoveDown_Action(node)),
                (MoveUpLeft_Action(node)),(MoveUpRight_Action(node)),(MoveDownLeft_Action(node)),(MoveDownRight_Action(node))]

    # Returns the parent node for a given node    
    def get_parent(self,node):
        return self.nodes[node][0]

class Djkstra(Map,Node):
    
    def __init__(self):
        super().__init__()
        super(Map,self).__init__()
        # Creating the map and obstacle map
        self.map,self.obs_map=self.create_map()
    
        # Inserting obstacle nodes into the nodes dictionary
        self.insert_node()

        # Getting user inputs
        self.get_user_inputs()
    
    # Djkstra function to generate the heap tree graph of the nodes using Djkstra's Algorithm
    def djkstra_algorithm(self):
        open_list=[]
        closed_list=set()                   
        explored_nodes=set()                
        heapq.heappush(open_list,(0,self.start_node))
        while open_list:
            current_distance,current_node=heapq.heappop(open_list)
            if current_node not in explored_nodes:
                explored_nodes.add(current_node)        
                    
            closed_list.add(current_node)                   

            if self.is_goal(current_node):
                return current_node
            
            for action in self.Actions(current_node):
                new_node,cost=action
                if new_node is not None and new_node not in closed_list:
                    if new_node not in explored_nodes:
                        new_cost=current_distance + cost
                        heapq.heappush(open_list,(new_cost,new_node))
                        explored_nodes.add(new_node)
                        self.insert_node(new_cost,new_node,current_node)
                    else:
                        for i in range(len(open_list)):
                            if open_list[i][1]==new_node:
                                new_cost=current_distance+cost
                                if open_list[i][0]>new_cost:
                                    open_list[i]=(new_cost,new_node)
                                    self.insert_node(new_cost,new_node,current_node)
        return None

    # Function to check if the current node is the goal node
    def is_goal(self,node):
        return node==self.end_node
    
    # Returns a path from the end_node to the start_node
    def construct_path(self):
        # Searching starts here
        path=[self.djkstra_algorithm()]
        total_cost=self.nodes[path[0]][1]
        parent=self.get_parent(path[0])
        while parent is not None:
            path.append(parent)
            parent = self.get_parent(parent)
        path.reverse()
        print("\nPath Found.")
        return path,total_cost

    # Getting user inputs
    def get_user_inputs(self):
        input_flag=True
        print("Consider clearance of 5mm.\nPlease enter the start and end nodes in the format 0 1 for (0,1)")
        while input_flag:
            s_node=input("Start node:")
            try:
                x,y=s_node.split()
                x,y=int(x),self.height-int(y)
            except:
                print("Please enter 2 valid integer coordinates.")
                continue
            if x not in range(self.width) or y not in range(self.height):
                print("Please enter valid coordinates.")
                continue
            if self.obs_map[y][x]==0:
                print("Please enter a valid start node(Node in obstacle place).")
            else:
                input_flag=False
                self.start_node=(x,y)
                self.insert_node(None,self.start_node,None)
    
        input_flag=True
        while input_flag:
            e_node=input("End node:")
            try:
                x,y=e_node.split()
                x,y=int(x),self.height-int(y)
            except:
                print("Please enter valid integer coordinates.")
                continue
            if x not in range(self.width) or y not in range(self.height) or (x,y)==self.start_node:
                print("Please enter valid coordinates.")
                continue
            if self.obs_map[y][x]==0:
                print("Please enter a valid end node(Node in obstacle place).")
            else:
                input_flag=False
                self.end_node=(x,y)

# Main function
if __name__ == "__main__":
    start_time = time.time()

    # Creating the object of the class    
    djkstra=Djkstra()

    # Generating the path and the total cost
    path,total_cost=djkstra.construct_path()   
    
    # Generating the path map
    djkstra.generate_path_map()
    
    # Printing the total cost and the path
    end_time = time.time()
    print(f"Time taken: {(end_time-start_time)/60} minutes.")
    print("\nPath cost: ",total_cost)
    print("\nPath: ",path)
