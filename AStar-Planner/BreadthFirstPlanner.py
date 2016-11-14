from collections import deque
import copy
import numpy
import time
import matplotlib.pyplot as pl
import IPython
import HerbEnvironment
class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        #IPython.embed()

        
    def Plan(self, start_config, goal_config):


        # visited : dictionary {node_id : parent_id}
        visited = {}
    	robot_env = self.planning_env.robot.GetEnv()
    	table = robot_env.GetKinBody('conference_table')
    	orig_tf = self.planning_env.robot.GetTransform()
    	# current_config[0][3] = start_config[0]
    	# current_config[1][3] = start_config[1]

        if self.visualize:
        	self.planning_env.InitializePlot(goal_config)

        plan = []
        Q = deque()

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id  = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        Q.append(start_id)
	print goal_id
        visited.update({start_id : 0})
        for_plot_parent = start_config
        for_plot_successor = start_config
	counter = 0
	counter2 = 0
        num=0

        start_time=time.time()
        while(Q):

        	parent_id = Q.popleft()
        	#for_plot_parent_id = visited.get(parent_id)
        	#if for_plot_parent_id != 0:
        	#	for_plot_parent = self.planning_env.discrete_env.NodeIdToConfiguration(for_plot_parent_id)

        	#	for_plot_successor = self.planning_env.discrete_env.NodeIdToConfiguration(parent_id)
            	# Plot edge between parent and valid successors
		#if type(self.planning_env) is not HerbEnvironment.HerbEnvironment:
            	#	self.planning_env.PlotEdge(for_plot_parent, for_plot_successor)
            # check if the goal is popped out
        	if parent_id == goal_id :
        		break
        	#
                num+=1
        	successors_id = self.planning_env.GetSuccessors(parent_id)
		#IPython.embed()
        	for successor_id in successors_id :

        		#print successors_id
        		parent_config     = self.planning_env.discrete_env.NodeIdToConfiguration(parent_id)
        		successor_config  = self.planning_env.discrete_env.NodeIdToConfiguration(successor_id)
        		# check if the successor is the goal
        		# print parent_id
        		# if successor_id == goal_id :
        		# 	visited.update({successor_id : parent_id})
        		# 	break
        		# check validity
        		temp_tf = copy.copy(orig_tf)
        		temp_tf[0][3] = successor_config[0] 
        		temp_tf[1][3] = successor_config[1]
        		self.planning_env.robot.SetTransform(temp_tf)

        		if robot_env.CheckCollision(self.planning_env.robot, table) :

        			self.planning_env.robot.SetTransform(orig_tf)
        		else:
        			self.planning_env.robot.SetTransform(orig_tf)
				
        			if (visited.has_key(successor_id)) is False :
        				# visiteupdate visited and Q
        				visited.update({successor_id : parent_id})
        				Q.append(successor_id)
					#print successor_id

        # build the path
        this_id = goal_id
	#IPython.embed()
        while(visited.get(this_id) != 0 and visited.get(this_id) != None ) :
        	this_config = self.planning_env.discrete_env.NodeIdToConfiguration(this_id)
        	plan.append(this_config)
        	this_id = visited.get(this_id) 	
                #print this_id, 'trying'
	plan.append(start_config)
	plan.reverse()
	
	nodelen = len(visited)


        path_length=0
	for i in range(len(plan)-1):
            if self.visualize:
                self.planning_env.PlotEdge(plan[i],plan[i+1])
            	
            node_i=self.planning_env.discrete_env.ConfigurationToNodeId(plan[i])
	    node_i1=self.planning_env.discrete_env.ConfigurationToNodeId(plan[i+1])
   	    path_length+=self.planning_env.ComputeDistance(node_i,node_i1)
            
        print "PATH LENGTH", path_length
        print 'RUN TIME', time.time()-start_time
        print 'NODES EXPANDED', num
        print 'PLAN LENGTH',len(plan)

        return plan
