#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

##################################################################
######################## HELPER FUNCTIONS ########################
##################################################################
def cornered(state, box):
    '''Checks if box is stuck in corner, but not at goal state'''

    up_blocked = (box[1] == 0) or ((box[0], box[1] + 1) in state.obstacles)
    down_blocked = (box[1] == state.height - 1) or ((box[0], box[1] - 1) in state.obstacles) 
    left_blocked = (box[0] == 0) or ((box[0] - 1, box[1]) in state.obstacles)
    right_blocked = (box[0] == state.width - 1) or ((box[0] + 1, box[1]) in state.obstacles)

    return (up_blocked or down_blocked) and (left_blocked or right_blocked) 

####################################################################
######################## SOKOBAN HEURISTICS ########################
####################################################################
def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.
    
    result = 0
    for box in state.boxes:
        if box in state.storage:
            continue
        
        # Check if box is stuck in corner
        if cornered(state, box):
            return float('inf')

        # Find the closest storage point for the box
        min_distance = float('inf')
        for storage in state.storage:
            distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
            if distance < min_distance:
                min_distance = distance
        result += min_distance
        
        # Find the closest robot to the box
        min_distance = float('inf')
        for robot in state.robots:
            distance = abs(box[0] - robot[0]) + abs(box[1] - robot[1])
            if distance < min_distance:
                min_distance = distance
        result += min_distance
    
    return result

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    result = 0
    for box in state.boxes:
        # Find the closest storage point for the box
        min_distance = float('inf')
        for storage in state.storage:
            distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
            if distance < min_distance:
                min_distance = distance
        if min_distance == float('inf'):
            return float('inf')
        result += min_distance
        
    return result

##################################################################
###################### SEARCH ALGORITHMS #########################
##################################################################
def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval

def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se = SearchEngine('custom', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn, fval_function=wrapped_fval_function)
    costbound = (float('inf'), float('inf'), float('inf'))
    return se.search(timebound, costbound)

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    endtime = os.times()[0] + timebound
    se = SearchEngine('custom', 'full')
    
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, sokoban_goal_state, heur_fn, fval_function=wrapped_fval_function)
    
    best_goal = False
    best_stat = False
    costbound = (float('inf'), float('inf'), float('inf'))
    
    timeleft = endtime - os.times()[0]
    while timeleft > 0:
        goal, stat = se.search(timeleft, costbound)
        if goal and (goal.gval < costbound[2]):
            costbound = (float('inf'), float('inf'), goal.gval)
            best_goal = goal
            best_stat = stat
        weight *= 0.8
        se.fval_function = (lambda sN: fval_function(sN, weight))
        timeleft = endtime - os.times()[0]
    
    return best_goal, best_stat

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    endtime = os.times()[0] + timebound
    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    
    best_goal = False
    best_stat = False
    costbound = (float('inf'), float('inf'), float('inf'))
    
    timeleft = endtime - os.times()[0]
    while timeleft > 0:
        goal, stat = se.search(timeleft, costbound)
        if goal and (goal.gval < costbound[0]):
            costbound = (goal.gval, float('inf'), float('inf'))
            best_goal = goal
            best_stat = stat
        timeleft = endtime - os.times()[0]
    
    return best_goal, best_stat

