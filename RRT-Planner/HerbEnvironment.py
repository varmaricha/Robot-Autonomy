import numpy
import random
from time import time

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.25

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def collision_checker(self,config):
    	joints = self.robot.GetActiveDOFIndices();
    	with self.robot.GetEnv():
    		self.robot.SetDOFValues(config, joints)
    	return self.robot.GetEnv().CheckCollision(self.robot, self.table) or self.robot.CheckSelfCollision()

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits();
        for i in xrange(len(config)):
        	inCollision = True
        	while inCollision:
        		config[i] = random.uniform(lower_limits[i], upper_limits[i])
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
    	
    	x = [numpy.linspace(start_config[i], end_config[i], number_of_steps) for i in xrange(len(self.robot.GetActiveDOFIndices()))]
    	config_to_return = None

    	for i in xrange(1,number_of_steps):
    		temp_config = numpy.asarray([x[j][i] for j in xrange(len(x))])
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
