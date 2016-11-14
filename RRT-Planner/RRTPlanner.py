import numpy
from RRTTree import RRTTree
from random import random

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        #plan.append(start_config)
        new_config = start_config
        distance = 100
        while  distance > epsilon:
            r = random()
            if r > self.planning_env.p:
                target_config = self.planning_env.GenerateRandomConfiguration()
            else:
                target_config = goal_config
            current_id, current_config = tree.GetNearestVertex(target_config)
            new_config = self.planning_env.Extend(current_config, target_config)
            if new_config == None:
                pass
            else:
                new_id = tree.AddVertex(new_config)
                tree.AddEdge(current_id, new_id)
                #self.planning_env.PlotEdge(tree.vertices[current_id], tree.vertices[new_id])
                distance = self.planning_env.ComputeDistance(new_config, goal_config)
            



        plan = []

        while new_id != 0:
            new_id = tree.edges[new_id] 
            prev_node = tree.vertices[new_id]
            plan.insert(0,prev_node)
        
        plan.append(goal_config)

        return plan
