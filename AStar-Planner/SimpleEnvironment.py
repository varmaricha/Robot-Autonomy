import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import copy

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.resolution = resolution
        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []
        tentative_successors =[]
        robot_env = self.robot.GetEnv()
        conf_table = robot_env.GetKinBody('conference_table')
        
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        this_node_coord  = self.discrete_env.NodeIdToGridCoord(node_id)
        x=this_node_coord[0]
        y=this_node_coord[1]
        tentative_successors=[[x-1,y-1],[x,y-1],[x+1,y-1],[x-1,y],[x+1,y],[x-1,y+1],[x,y+1],[x+1,y+1]]

        for i in range(len(tentative_successors)):
            if tentative_successors[i][0]>=0 and tentative_successors[i][0]<self.discrete_env.num_cells[0] and tentative_successors[i][1]>=0 and tentative_successors[i][1]<self.discrete_env.num_cells[1]:
                tf = self.robot.GetTransform()
                tf_robot = copy.copy(tf)

                go_to=self.discrete_env.GridCoordToConfiguration(tentative_successors[i])
                tf[0][3] = go_to[0]
                tf[1][3] = go_to[1]
                self.robot.SetTransform(tf)
                if not robot_env.CheckCollision(self.robot, conf_table):
                    successors.append(self.discrete_env.GridCoordToNodeId(tentative_successors[i]))
                # TODO: Here you will implement a function that looks
                #  up the configuration associated with the particular node_id
                #  and return a list of node_ids that represent the neighboring
                #  nodes

                # try dummy way in 2D first, 7D.....We will see

                #total_idx = self.discrete_env.num_cells[0]*self.discrete_env.num_cells[1] - 1
                self.robot.SetTransform(tf_robot)
             
        '''# left neighbor
        left_node_coord  = numpy.add(this_node_coord,[-1,0])
        left_node_id     = self.discrete_env.GridCoordToNodeId(left_node_coord)
        # right neighbor
        right_node_coord = numpy.add(this_node_coord,[1,0])
        right_node_id    = self.discrete_env.GridCoordToNodeId(right_node_coord)
        # up neighbor
        up_node_coord    = numpy.add(this_node_coord,[0, -1])
        up_node_id       = self.discrete_env.GridCoordToNodeId(up_node_coord)
        # down neighbor
        down_node_coord  = numpy.add(this_node_coord,[0, 1])
        down_node_id     = self.discrete_env.GridCoordToNodeId(down_node_coord)'''


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
        
        start_grid=self.discrete_env.NodeIdToGridCoord(start_id)
        goal_grid=self.discrete_env.NodeIdToGridCoord(goal_id)

#        start_grid=self.discrete_env.NodeIdToConfiguration(start_id)
#        goal_grid=self.discrete_env.NodeIdToConfiguration(goal_id)

        
        cost=abs(start_grid[0]-goal_grid[0]) + abs(start_grid[1]-goal_grid[1])

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        
    def IsValid(self, coord):
    	upper_bound_coord = self.discrete_env.ConfigurationToGridCoord([5,5])
        lower_bound_coord = self.discrete_env.ConfigurationToGridCoord([-5,-5])
        if coord[0] > upper_bound_coord[0] :
        	return False
        if coord[0] < lower_bound_coord[0] :
        	return False
        if coord[1] > upper_bound_coord[1] :
        	return False
        if coord[1] < lower_bound_coord[1] :
        	return False
        return True
