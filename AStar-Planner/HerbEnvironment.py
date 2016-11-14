import numpy
from DiscreteEnvironment import DiscreteEnvironment
from operator import add
import copy

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []
        #tentative_successors =[]
        robot_env = self.robot.GetEnv()
        conf_table = robot_env.GetKinBody('conference_table')
        
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        this_node_coord  = self.discrete_env.NodeIdToGridCoord(node_id)
        this_node_coord=list(this_node_coord)
        tentative_successor1=copy.copy(this_node_coord)
        tentative_successor2=copy.copy(this_node_coord)
        #tentative_successor=copy.deepcopy(this_node_coord)                

     
        #tentative_successor1=map(add, this_node_coord, add1)
        #tentative_successor2=map(add, this_node_coord, sub1)
        #tentative_successors=[tentative_successor1,tentative_successor2]    

        for i in range(len(this_node_coord)): 

                for k in range(2):
                    if k==0:
                        tentative_successor=copy.copy(tentative_successor1)
			tentative_successor[i]=tentative_successor1[i]+1
                    else:
                        tentative_successor=copy.copy(tentative_successor2)
			tentative_successor[i]=tentative_successor2[i]-1

                    if tentative_successor[i]>=0:
                      
                        tf_robot=self.robot.GetActiveDOFValues() 
                                
               
                        go_to=self.discrete_env.GridCoordToConfiguration(tentative_successor)
                        self.robot.SetActiveDOFValues(go_to)

                        if self.robot.GetEnv().CheckCollision(self.robot,conf_table) or self.robot.GetEnv().CheckCollision(self.robot,self.robot):

                            continue
                        else:
                            successors.append(self.discrete_env.GridCoordToNodeId(tentative_successor))
                # TODO: Here you will implement a function that looks
                #  up the configuration associated with the particular node_id
                #  and return a list of node_ids that represent the neighboring
                #  nodes

               
                        #self.robot.SetActiveDOFValues(tf_robot)

       
        return successors


    

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        end_config   = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
        dist = numpy.linalg.norm((end_config - start_config))

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        start_grid=self.discrete_env.NodeIdToGridCoord(start_id)
        goal_grid=self.discrete_env.NodeIdToGridCoord(goal_id)

#        start_grid=self.discrete_env.NodeIdToConfiguration(start_id)
#        goal_grid=self.discrete_env.NodeIdToConfiguration(goal_id)

        for i in range(len(start_grid)):
            cost+=abs(start_grid[i]-goal_grid[i]) 
        
        return cost

    def IsValid(self, coord):
    	upper_bound_coord = self.discrete_env.ConfigurationToGridCoord(self.upper_limits)
        lower_bound_coord = self.discrete_env.ConfigurationToGridCoord(self.lower_limits)
        if coord[0] > upper_bound_coord[0] :
        	return False
        if coord[0] < lower_bound_coord[0] :
        	return False
        if coord[1] > upper_bound_coord[1] :
        	return False
        if coord[1] < lower_bound_coord[1] :
        	return False
        if coord[2] > upper_bound_coord[2] :
        	return False
        if coord[2] < lower_bound_coord[2] :
        	return False	
        if coord[3] > upper_bound_coord[3] :
        	return False
        if coord[3] < lower_bound_coord[3] :
        	return False
        if coord[4] > upper_bound_coord[4] :
        	return False
        if coord[4] < lower_bound_coord[4] :
        	return False	
        if coord[5] > upper_bound_coord[5] :
        	return False
        if coord[5] < lower_bound_coord[5] :
        	return False
        if coord[6] > upper_bound_coord[6] :
        	return False
        if coord[6] < lower_bound_coord[6] :
        	return False
        return True


