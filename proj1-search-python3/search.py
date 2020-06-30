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
import time
from collections import *

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
        return self.getStartState()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        return state == self.isGoalState()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        return self.getSuccessors(state)

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        return self.getCostOfActions(actions)


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
    "*** YOUR CODE HERE ***"
    stack = util.Stack()
    visited = set()

    stack.push((problem.getStartState(), list()))

    while not stack.isEmpty():
        current = stack.pop()

        if problem.isGoalState(current[0]):
            return current[1]

        successors = problem.getSuccessors(current[0])
        visited.add(current[0])

        for s in successors:
            if s[0] not in visited:
                path = list(current[1])
                path.append(s[1])
                stack.push((s[0], path))

    print("No path found")
    return list()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queue = util.Queue()
    visited = list()

    queue.push((problem.getStartState(), list()))

    while not queue.isEmpty():
        node, actions = queue.pop()

        if problem.isGoalState(node):
            return actions

        successors = problem.getSuccessors(node)
        if node not in visited:
            visited.append(node)

        for n, a, c in successors:
            if n not in visited and n not in (node for node, action in queue.list):
                path = list(actions)
                path.append(a)
                queue.push((n, path))

    print("No path found")
    return list()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    pq = util.PriorityQueue()
    visited = set()

    pq.push((problem.getStartState(), list()), 0)

    while not pq.isEmpty():
        node, actions = pq.pop()

        if node not in visited:
            if problem.isGoalState(node):
               return actions

            visited.add(node)
            successors = problem.getSuccessors(node)

            for n, a, _ in successors:
                if n not in visited:
                    n_actions = actions + [a]
                    pq.push((n, n_actions), problem.getCostOfActions(n_actions))


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    if problem.isGoalState(problem.getStartState()):
        return []

    pq = util.PriorityQueue()
    visited = list()

    pq.push((problem.getStartState(), list(), 0), 0)

    while not pq.isEmpty():
        node, actions, prev_cost = pq.pop()

        if node not in visited:
            visited.append(node)
            if problem.isGoalState(node):
               return actions

            successors = problem.getSuccessors(node)

            for n, a, c in successors:
                if n not in visited:
                    n_actions = actions + [a]
                    cost = prev_cost + c
                    pq.push((n, n_actions, cost), problem.getCostOfActions(n_actions) + heuristic(n, problem))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
