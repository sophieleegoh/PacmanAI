# myAgents.py
# ---------------
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

from game import Agent
from searchProblems import PositionSearchProblem

import util
import time
import search
import random

"""
IMPORTANT
`agent` defines which agent you will use. By default, it is set to ClosestDotAgent,
but when you're ready to test your own agent, replace it with MyAgent
"""
def createAgents(num_pacmen, agent='MyAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]

class MyAgent(Agent):
    """
    Implementation of your agent.
    """

    def getAction(self, state):
        """
        Returns the next action the agent will take
        """

        "*** YOUR CODE HERE ***"

        problem = AnyFoodSearchProblem(state, self.index)
        pacmen_locations = state.getPacmanPositions()
        current = state.getPacmanPosition(self.index)

        near = False
        index = 0
        for location in pacmen_locations:
            if index != self.index:
                if location[0] in {current[0] - 1, current[0], current[0] + 1} and location[1] in (current[1] - 1, current[1], current[1] + 1):
                    near = True
                    print(location)
                    break
                if len(self.moves) > 0:
                    if location == state.generatePacmanSuccessor(self.moves[0], self.index):
                        near = True
                        print(location)
                        break
            index += 1

        #caching bfs
        if len(self.moves) == 0:
            self.moves = search.bfs(problem)
            print(self.moves)
            self.target = self.getTarget(state, self.moves)

        if near:
            self.moves = []
            print(len(self.moves))
            if len(self.moves) < 2 and len(self.moves) > 0:
                return self.moves.pop(0)

            if self.priority in state.getLegalPacmanActions(self.index):
                return self.priority
            return random.choice(state.getLegalPacmanActions(self.index))

        return self.moves.pop(0)


    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """

        "*** YOUR CODE HERE"

        #raise NotImplementedError()
        self.moves = []

        if self.index % 4 == 0:
            self.priority = 'North'
        elif self.index % 4 == 1:
            self.priority = 'East'
        elif self.index % 4 == 2:
            self.priority = 'South'
        else:
            self.priority = 'West'

        self.target = ()


    def getTarget(self, state, moves):
        x, y = state.getPacmanPosition(self.index)
        for move in moves:
            if move == 'North':
                y += 1
            elif move == 'East':
                x += 1
            elif move == 'South':
                y -= 1
            else:
                x -= 1

        return (x, y)

#NOT USED
    def search1(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)


        "*** YOUR CODE HERE ***"
        return search.bfs(problem)

    def search2(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)

        top, right = self.walls.height - 2, self.walls.width - 2
        goals = ((1, top), (right, 1), (top, right))



        "*** YOUR CODE HERE ***"
        return search.aStarSearch(problem)

"""
Put any other SearchProblems or search methods below. You may also import classes/methods in
search.py and searchProblems.py. (ClosestDotAgent as an example below)
"""

class ClosestDotAgent(Agent):

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)


        "*** YOUR CODE HERE ***"
        return search.bfs(problem)

    def getAction(self, state):
        return self.findPathToClosestDot(state)[0]

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState, agentIndex):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition(agentIndex)
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        "*** YOUR CODE HERE ***"
        distance, goal = min([(util.manhattanDistance(state, goal), goal) for goal in self.food.asList()])

        if state == goal:
            return True
        return False

