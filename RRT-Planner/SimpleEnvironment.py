import numpy
import matplotlib.pyplot as pl
import random
from time import time

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.1

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    
    def collision_checker(self,config):
    	x = config[0]
    	y = config[1]
    	Tz = numpy.eye(4)
    	
    	Tz[0][3] = x
    	Tz[1][3] = y
    	with self.robot.GetEnv():
            self.robot.SetTransform(Tz)
    	return self.robot.GetEnv().CheckCollision(self.robot, self.table)



    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        inCollision = True
        while inCollision:
        	config[0] = random.uniform(lower_limits[0], upper_limits[0])
        	config[1] = random.uniform(lower_limits[1], upper_limits[1])
        	inCollision = self.collision_checker(config) 
        	
        
        
        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #

        return numpy.linalg.norm(end_config - start_config)

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        
    	dist = self.ComputeDistance(start_config, end_config)
    	number_of_steps = int(dist*10)
    	x = numpy.linspace(start_config[0], end_config[0], number_of_steps)
    	y = numpy.linspace(start_config[1], end_config[1], number_of_steps)
    	config_to_return = None

    	for i in xrange(1,number_of_steps):
    		temp_config = numpy.array([x[i], y[i]])
    		inCollision = self.collision_checker(temp_config)
    		if inCollision:
    			return config_to_return
    		else:
    			config_to_return = temp_config
    	return config_to_return



    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        t = time()
        while time() - t < timeout:
            idx1 = random.randint(0,len(path)-1)
            idx2 = random.randint(idx1,len(path)-1)
            q_new = self.Extend(path[idx1], path[idx2])
            if q_new != None:
                if numpy.array_equal(q_new,path[idx2]):
                    path[idx1+1:idx2] = []
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
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

