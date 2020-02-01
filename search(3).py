# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import searchAgents

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    from game import Directions
    n = Directions.NORTH
    s = Directions.SOUTH
    e = Directions.EAST
    w = Directions.WEST

    fringe = util.Stack()
    fringe.push([(problem.getStartState(), None, None)])
    closed = []
    path = []
    while not fringe.isEmpty():
        curr = fringe.pop()  # this is of the format [(state, dir, cost), ... , (state, dir, cost)]
        if problem.isGoalState(curr[-1][0]):
            path = curr
            break
        elif curr[-1][0] not in closed:
            closed.append(curr[-1][0]) # adding the last bit of the current path to the closed set
            succs = problem.getSuccessors(curr[-1][0])[::-1]
            for succ in succs:
                potentialPath = curr + [succ]
                fringe.push(potentialPath)
    # FIXME possibly remove ?????
    if path == []:
        return "Fail"

    # to construct a returnable list of actions from the current path:
    actions = []
    for tup in path:
        direction = tup[1]
        if direction == "North":
            actions.append(n)
        elif direction == "South":
            actions.append(s)
        elif direction == "East":
            actions.append(e)
        elif direction == "West":
            actions.append(w)
        elif direction != None:
            actions.append(direction)
    return actions


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    from game import Directions
    n = Directions.NORTH
    s = Directions.SOUTH
    e = Directions.EAST
    w = Directions.WEST

    fringe = util.Queue()
    if isinstance(problem, searchAgents.CornersProblem):
        fringe.push([problem.getStartState()])
    else:
        fringe.push([(problem.getStartState(), None, None)])
    closed = []
    path = []
    while not fringe.isEmpty():
        curr = fringe.pop()  # this is of the format [(state, dir, cost), ... , (state, dir, cost)]
                             # or w/extra booleans if CornersProblem

        # ( (x,y), dir, cost, False, False, False, False )
        # consider changing how we add nodes to closed
        # (since we can now re-add things in pursuit of finding a path to all four corners ?)
        if len(curr[0]) == 7:
            if problem.isGoalState(curr[-1]):
                path = curr
                break
            elif curr[-1] not in closed:
                closed.append(curr[-1])
                succs = problem.getSuccessors(curr[-1])[::-1]
                for succ in succs:
                    potentialPath = curr + [succ]
                    fringe.push(potentialPath)
        else:
            if problem.isGoalState(curr[-1][0]):
                path = curr
                break
            elif curr[-1][0] not in closed:
                closed.append(curr[-1][0])
                succs = problem.getSuccessors(curr[-1][0])[::-1]
                for succ in succs:
                    potentialPath = curr + [succ]
                    fringe.push(potentialPath)
        # ((x1, y1), None, None)
        # ((x1, y1), dir, cost)

        # if (len(curr[-1]) == 3 and curr[-1][0] not in closed) or (curr[-1] not in closed):
        #     closed.append(curr[-1]) # adding the last bit of the current path to the closed set
        #     if len(curr[0]) == 7:
        #         succs = problem.getSuccessors(curr[-1])[::-1]
        #     else:
        #         succs = problem.getSuccessors(curr[-1][0])[::-1]
        #     for succ in succs:
        #         potentialPath = curr + [succ]
        #         fringe.push(potentialPath)
    # FIXME possibly remove ?????
    if path == []:
        return "Fail"

    # to construct a returnable list of actions from the current path:
    actions = []
    for tup in path:
        direction = tup[1]
        if direction == "North":
            actions.append(n)
        elif direction == "South":
            actions.append(s)
        elif direction == "East":
            actions.append(e)
        elif direction == "West":
            actions.append(w)
        elif direction != None:
            actions.append(direction)
    return actions

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    from game import Directions
    n = Directions.NORTH
    s = Directions.SOUTH
    e = Directions.EAST
    w = Directions.WEST

    fringe = util.PriorityQueue()
    fringe.push([[(problem.getStartState(), None, None)], 0], 0)
    closed = []
    path = []
    while not fringe.isEmpty():
        curr = fringe.pop()  # this is of the format [[(state, dir, cost), ... , (state, dir, cost)], totalPathCost]
        if problem.isGoalState(curr[0][-1][0]):
            path = curr[0]
            break
        elif curr[0][-1][0] not in closed:
            closed.append(curr[0][-1][0]) # adding the last bit of the current path to the closed set
            succs = problem.getSuccessors(curr[0][-1][0])[::-1]
            for succ in succs:
                potentialPath = curr[0] + [succ]
                potentialCost = curr[1] + succ[2]
                potential = [potentialPath, potentialCost]
                fringe.push(potential, potentialCost)
    # FIXME possibly remove ?????
    if path == []:
        return "Fail"

    # to construct a returnable list of actions from the current path:
    actions = []
    for tup in path:
        direction = tup[1]
        if direction == "North":
            actions.append(n)
        elif direction == "South":
            actions.append(s)
        elif direction == "East":
            actions.append(e)
        elif direction == "West":
            actions.append(w)
        elif direction != None:
            actions.append(direction)
    return actions

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    from game import Directions
    n = Directions.NORTH
    s = Directions.SOUTH
    e = Directions.EAST
    w = Directions.WEST

    fringe = util.PriorityQueue()
    startState = problem.getStartState()
    startHeur = heuristic(startState, problem)
    if isinstance(problem, searchAgents.CornersProblem):
        fringe.push([[startState], 0, startHeur], startHeur)
    else:
        fringe.push([[(startState, None, None)], 0, startHeur], startHeur)
    closed = []
    path = []
    while not fringe.isEmpty():
        curr = fringe.pop()  # this is not of the format [[(state, dir, cost), ... , (state, dir, cost)], totalPathCost]
        if len(curr[0][0]) == 7:
            if problem.isGoalState(curr[0][-1]):
                path = curr[0]
                break
            elif curr[0][-1] not in closed:
                closed.append(curr[0][-1]) # adding the last bit of the current path to the closed set
                succs = problem.getSuccessors(curr[0][-1])[::-1]
                for succ in succs:
                    potentialPath = curr[0] + [succ]
                    potentialCost = curr[1] + succ[2]
                    potentialHeur = heuristic(succ[0], problem) # maybe just succ
                    potential = [potentialPath, potentialCost, potentialHeur]
                    fringe.push(potential, potentialCost + potentialHeur)
        else:
            if problem.isGoalState(curr[0][-1][0]):
                path = curr[0]
                break
            elif curr[0][-1][0] not in closed:
                closed.append(curr[0][-1][0]) # adding the last bit of the current path to the closed set
                succs = problem.getSuccessors(curr[0][-1][0])[::-1]
                for succ in succs:
                    potentialPath = curr[0] + [succ]
                    potentialCost = curr[1] + succ[2]
                    potentialHeur = heuristic(succ[0], problem)
                    potential = [potentialPath, potentialCost, potentialHeur]
                    fringe.push(potential, potentialCost + potentialHeur)

    # FIXME possibly remove ?????
    if path == []:
        return "Fail"

    # to construct a returnable list of actions from the current path:
    actions = []
    for tup in path:
        direction = tup[1]
        if direction == "North":
            actions.append(n)
        elif direction == "South":
            actions.append(s)
        elif direction == "East":
            actions.append(e)
        elif direction == "West":
            actions.append(w)
        elif direction != None:
            actions.append(direction)
    return actions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
