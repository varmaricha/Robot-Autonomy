from collections import OrderedDict
from operator import itemgetter
import time


class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()



    def AStar(self,start,goal):

	if self.visualize and hasattr(self.planning_env, "InitializePlot"):
            self.planning_env.InitializePlot(goal)        	

	closedSet=[]
        start=self.planning_env.discrete_env.ConfigurationToNodeId(start)
        goal=self.planning_env.discrete_env.ConfigurationToNodeId(goal)
        #print start
        #print goal
	openSet=[start]
        parent = {}

        gscore={}
        gscore[start]=0

        fscore={}
        fscore[start]=self.planning_env.ComputeHeuristicCost(start, goal)
        #print fscore
        #print openSet
        num=0
	
        while openSet:

            open_fscore=dict((k, fscore[k]) for k in openSet)
	    open_fscore=OrderedDict(sorted(open_fscore.items(), key=itemgetter(1)))
            open_fscore_list=open_fscore.keys()
            #print open_fscore_list
            current=open_fscore_list[0]
            

            #current=min(fscore,key=lambda k:fscore[k])

            if current == goal:
		return self.reconstruct_path(parent,goal), num

            num+=1
            openSet.remove(current)
            closedSet.append(current)

            neighbors= self.planning_env.GetSuccessors(current)
            for node in neighbors:
                if node in closedSet:
                    continue 

                tentative_gscore=gscore[current]+self.planning_env.ComputeDistance(current,node)

                if node not in openSet:
                    openSet.append(node)
		elif tentative_gscore>=gscore[node]:
                    continue

                parent[node]=current
                gscore[node]=tentative_gscore
                fscore[node]=gscore[node] + self.planning_env.ComputeHeuristicCost(node,goal)


        return None 


    def reconstruct_path(self,parent, current):

        total_path = [current]
        parent_keys=parent.keys()
        while current in parent_keys:
            current = parent[current]
            total_path.append(current)
        return total_path


    def Plan(self, start_config, goal_config):

        plan = []
        start_time=time.time()
        plan,num=self.AStar(start_config, goal_config)


        plan.reverse()
        path=[]
        
        for i in range(len(plan)): 
            path.append(self.planning_env.discrete_env.NodeIdToConfiguration(plan[i]))


        path_length=0
	for i in range(len(plan)-1):
            if self.visualize:
                self.planning_env.PlotEdge(path[i],path[i+1])
            	    #self.planning_env.PlotEdge(plan[i],plan[i+1])
		    #dist+= numpy.sqrt(numpy.square(plan[i][0]-plan[i+1][0])+numpy.square(plan[i][1]-plan[i+1][1]))
            node_i=self.planning_env.discrete_env.ConfigurationToNodeId(path[i])
	    node_i1=self.planning_env.discrete_env.ConfigurationToNodeId(path[i+1])
   	    path_length+=self.planning_env.ComputeDistance(node_i,node_i1)
            
        print "PATH LENGTH", path_length
        print 'RUN TIME', time.time()-start_time
        print 'NODES EXPANDED', num
        print 'PLAN LENGTH',len(plan)
        
        return path


         

'''fscore_list=fscore.keys()
            open_score=[]
            for node in openSet:
		open_score.append=fscore[node]
            open_sorted=sorted(open_score)'''          


