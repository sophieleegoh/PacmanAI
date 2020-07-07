# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        foodList = newFood.asList()

        closest_food = float("infinity")

        for food in foodList:
            closest_food = min(closest_food, manhattanDistance(newPos, food))

        return successorGameState.getScore() + 1.0/closest_food


def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"
        actions = self.max_value(gameState, 0, 0)
        return actions[0]

    def max_value(self, state, index, depth):
        v = -float("infinity")
        actions_list = state.getLegalActions(index)
        a = "" #actual action taken

        if len(actions_list) == 0:
            return self.evaluationFunction(state)

        for action in actions_list:
            successors = state.generateSuccessor(index, action)
            weight = self.min_max(successors, index + 1, depth)

            if weight > v:
                v = weight
                a = action

        return [a, v]

    def min_value(self, state, index, depth):
        v = float("infinity")
        actions_list = state.getLegalActions(index)
        a = "" #actual action taken

        if len(actions_list) == 0:
            return self.evaluationFunction(state)

        for action in actions_list:
            successors = state.generateSuccessor(index, action)
            weight = self.min_max(successors, index + 1, depth)

            if weight < v:
                v = weight
                a = action

        return [a, v]


    """
        If the index is 0, that means it is Pac man. Anything > 0 is a ghost
    """
    def min_max(self, state, index, depth):
        if index >= state.getNumAgents():
            depth += 1
            index = 0

        if depth == self.depth or state.isWin() or state.isLose():
            return self.evaluationFunction(state)

        if index == 0:
            return self.max_value(state, index, depth)[1]

        if index > 0:
            return self.min_value(state, index, depth)[1]



class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        actions = self.max_value(gameState, 0, 0, -float("infinity"), float("infinity"))
        return actions[0]

    def max_value(self, state, index, depth, a, b):
        v = -float("infinity")
        actions_list = state.getLegalActions(index)
        m = "" #actual action taken

        if len(actions_list) == 0:
            return self.evaluationFunction(state)

        for action in actions_list:
            successors = state.generateSuccessor(index, action)
            weight = self.min_max(successors, index + 1, depth, a, b)

            if weight > v:
                v = weight
                m = action

            if weight > b:
                break

            a = max(a, weight)
        return [m, v]

    def min_value(self, state, index, depth, a, b):
        v = float("infinity")
        actions_list = state.getLegalActions(index)
        m = "" #actual action taken

        if len(actions_list) == 0:
            return self.evaluationFunction(state)

        for action in actions_list:
            successors = state.generateSuccessor(index, action)
            weight = self.min_max(successors, index + 1, depth, a, b)

            if weight < v:
                v = weight
                m = action

            if weight < a:
                break

            b = min(b, weight)
        return [m, v]


    """
        If the index is 0, that means it is Pac man. Anything > 0 is a ghost
    """
    def min_max(self, state, index, depth, a, b):
        if index >= state.getNumAgents():
            depth += 1
            index = 0

        if depth == self.depth or state.isWin() or state.isLose():
            return self.evaluationFunction(state)

        if index == 0:
            return self.max_value(state, index, depth, a, b)[1]

        if index > 0:
            return self.min_value(state, index, depth, a, b)[1]

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        actions = self.max_value(gameState, 0, 0)
        return actions[0]

    def max_value(self, state, index, depth):
        v = -float("infinity")
        actions_list = state.getLegalActions(index)
        m = actions_list[0] #actual action taken

        if len(actions_list) == 0:
            return self.evaluationFunction(state)

        for action in actions_list:
            successors = state.generateSuccessor(index, action)
            weight = self.value(successors, index + 1, depth)

            if weight > v:
                v = weight
                m = action

        return [m, v]

    def exp_value(self, state, index, depth):
        e = 0
        actions_list = state.getLegalActions(index)

        if len(actions_list) == 0:
            return self.evaluationFunction(state)

        probability = 1.0 / len(actions_list) # since there is an equal probability

        for action in actions_list:
            successors = state.generateSuccessor(index, action)
            weight = self.value(successors, index + 1, depth)

            e += weight * probability

        return [actions_list[-1], e]

    def value(self, state, index, depth):
        if index >= state.getNumAgents():
            depth += 1
            index = 0

        if depth == self.depth or state.isWin() or state.isLose():
            return self.evaluationFunction(state)

        if index == 0:
            return self.max_value(state, index, depth)[1]

        if index > 0:
            return self.exp_value(state, index, depth)[1]


def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"


    food = currentGameState.getFood().asList()
    position = currentGameState.getPacmanPosition()
    ghosts = currentGameState.getGhostStates()
    capsules = currentGameState.getCapsules()

    closestFood = float("infinity")
    for f in food:
        closestFood = min(closestFood, manhattanDistance(position, f))

    enemy_ghosts = []
    food_ghosts = []

    for g in ghosts:
        if g.scaredTimer:
            food_ghosts.append(g)
        else:
            enemy_ghosts.append(g)

    closestCapsule = float("infinity")
    for c in capsules:
        closestCapsule = min(closestCapsule, manhattanDistance(position, c))


    additionalScore = 0
    # Lots of remaining food is bad
    additionalScore -= currentGameState.getNumFood()

    if closestFood < 5:
        additionalScore += 100

    if closestCapsule < 2:
        additionalScore += 1000

    for ghost in food_ghosts:
        distance = manhattanDistance(position, ghost.getPosition())
        if distance < 5:
            additionalScore += 100000 * distance

    for ghost in enemy_ghosts:
        if manhattanDistance(position, ghost.getPosition()) < 4:
            additionalScore -= float("infinity")

    if currentGameState.isWin():
        additionalScore = float("infinity")

    #print(additionalScore)

    return currentGameState.getScore() + 1.0/additionalScore



# Abbreviation
better = betterEvaluationFunction
