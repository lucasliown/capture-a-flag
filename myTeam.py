# myTeam.py
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


from os import get_terminal_size
from typing import Type
from capture import GameState
from captureAgents import CaptureAgent
import random, util
from game import Directions, Actions
from random import choice


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='OffensiveAgent', second='defAgent', numTraining=0):
    """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.
  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

    # The following line is an example only; feel free to change it.
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class DummyAgent(CaptureAgent):
    """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

    def registerInitialState(self, gameState):
        """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).
    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)
    IMPORTANT: This method may run for at most 15 seconds.
    """

        '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
        CaptureAgent.registerInitialState(self, gameState)

        '''
    Your initialization code goes here, if you need any.
    '''

    def chooseAction(self, gameState):
        """
    Picks among actions randomly.
    """
        actions = gameState.getLegalActions(self.index)
        '''
    You should change this in your own agent.
    '''
        return random.choice(actions)


class OffensiveAgent(CaptureAgent):
    def registerInitialState(self,gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.start = gameState.getAgentPosition(self.index)
        walls = gameState.getWalls().asList()
        self.max_X = max(wall[0] for wall in walls)
        self.max_Y = max(wall[1] for wall in walls)
        self.mid_X = self.max_X / 2
        self.mid_Y = self.max_Y / 2
        self.depth = 2
        self.finalScore = 0
        wall = gameState.getWalls().height
        self.distance = wall/2
        self.count = len(self.getOpponents(gameState))
        self.observationHistory = []
        self.boundaryNew = self.boundaryPosition(gameState)
        if self.red:
            self.boundary = int(gameState.getWalls().width / 2) - 1
        else:
            self.boundary = int(gameState.getWalls().width / 2)

    def chooseAction(self, gameState):
        distance = 1000000000
        for opponent in self.getOpponents(gameState):
            if gameState.getAgentState(opponent).isPacman == False:
                if gameState.getAgentState(opponent).getPosition() is not None:
                   if gameState.getAgentState(opponent).scaredTimer == 0:
                        if self.getMazeDistance(gameState.getAgentPosition(self.index), gameState.getAgentState(opponent).getPosition()) < distance:
                            distance=self.getMazeDistance(gameState.getAgentPosition(self.index), gameState.getAgentState(opponent).getPosition())
        if distance < 3:
            new = self.getEscape(gameState)
            return new
        actions = gameState.getLegalActions(self.index)
        values = [self.evaluate(gameState, a) for a in actions]
        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]
        return random.choice(bestActions)

    def evaluate(self, gameState, action):
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

    def SortDistances(self, startingPoint, destination):
        allDistance = util.PriorityQueue()
        for i in destination:
            allDistance.push(i, self.getMazeDistance(i, startingPoint))
        return allDistance

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        allWalls = gameState.getWalls()
        allFood = self.getFood(gameState)
        allcapsules = gameState.getCapsules()
        pos = gameState.getAgentState(self.index).getPosition()
        newPos = successor.getAgentState(self.index).getPosition()
        x, y = gameState.getAgentState(self.index).getPosition()
        nextX, nextY = successor.getAgentState(self.index).getPosition()
        foods = allFood.asList()
        if action == Directions.STOP:
            features['stop'] = 1
        opponentStates = [successor.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
        opponentGhost = []
        opponentInvaders = []
        for opponentState in opponentStates:
            if opponentState.getPosition() != None:
                if opponentState.isPacman:
                    opponentInvaders.append(opponentState)
                elif not opponentState.isPacman:
                    opponentGhost.append(opponentState)
        # for cap in allcapsules:
        #     if nextX == cap[0] and nextY == cap[1]:
        #         if successor.getAgentState(self.index).isPacman:
        #             features["Capsule"] = 1.0
        for opponent in opponentInvaders:
            gPos = opponent.getPosition()
            ghostNext = Actions.getLegalNeighbors(gPos,allWalls)
            if opponent.scaredTimer > 0:
                if (nextX, nextY) == gPos:
                    features["eatMoreFood"] += 2
                elif (nextX, nextY) in ghostNext:
                    features["ghostScared"] += 1
                elif successor.getAgentState(self.index).isPacman:
                    features["ghostScared"] = 0
                    features["ghost"] += 1

        ghostCount = 0
        near = 10000
        scarCount = 0
        for opp in self.getOpponents(gameState):
            if gameState.getAgentState(opp).isPacman==False:
                ghostCount=ghostCount+1
            if gameState.getAgentState(opp).getPosition() is not None:
                di = self.getMazeDistance(gameState.getAgentState(opp).getPosition(),pos)
                if di < near:
                    near = di
            if gameState.getAgentState(opp).scaredTimer>0 and gameState.getAgentState(opp).isPacman==False:
                scarCount = scarCount+1
            if gameState.getAgentState(opp).isPacman:
                scarCount = scarCount + 1
        if near <= 3:
            if gameState.getAgentState(self.index).numCarrying >= 3:
                distance = self.SortDistances(pos, self.boundaryNew)
                sortest = distance.pop()
                dis = self.getMazeDistance(pos, sortest)
                distanceNew = self.SortDistances(newPos, self.boundaryNew)
                sort = distanceNew.pop()
                disst = self.getMazeDistance(newPos, sort)
                if disst < dis:
                    features["goHome"] = gameState.getAgentState(self.index).numCarrying
        elif ghostCount==0 or scarCount==2:
            if len(foods)<=2:
                distance = self.SortDistances(pos, self.boundaryNew)
                sortest = distance.pop()
                dis = self.getMazeDistance(pos,sortest)
                distanceNew = self.SortDistances(newPos, self.boundaryNew)
                sort = distanceNew.pop()
                disst = self.getMazeDistance(newPos, sort)
                if disst < dis:
                    features["goHome"] = gameState.getAgentState(self.index).numCarrying
        else:
            if gameState.getAgentState(self.index).numCarrying > 4:
                distance = self.SortDistances(pos, self.boundaryNew)
                sortest = distance.pop()
                dis = self.getMazeDistance(pos,sortest)
                distanceNew = self.SortDistances(newPos, self.boundaryNew)
                sort = distanceNew.pop()
                disst = self.getMazeDistance(newPos, sort)
                if disst < dis:
                    features["goHome"] = gameState.getAgentState(self.index).numCarrying
        nearGhost = None
        nearDistance = 1000
        ghost = self.getOpponents(gameState)
        for gh in ghost:
            if gameState.getAgentState(gh).isPacman:
                if gameState.getAgentState(gh).getPosition() is not None:
                    if self.getMazeDistance(gameState.getAgentState(gh).getPosition(), newPos)< nearDistance:
                        nearDistance = self.getMazeDistance(gameState.getAgentState(gh).getPosition(), newPos)
                        nearGhost = gh

        if gameState.getAgentState(self.index).isPacman == False:
            if gameState.getAgentState(self.index).scaredTimer == 0:
                if nearGhost is not None:
                    if self.getMazeDistance(gameState.getAgentState(nearGhost).getPosition(), pos) <= self.distance:
                        if self.getMazeDistance(gameState.getAgentState(nearGhost).getPosition(), pos) >self.getMazeDistance(gameState.getAgentState(nearGhost).getPosition(), newPos):
                                features["eatP"] = self.getMazeDistance(gameState.getAgentState(nearGhost).getPosition(), pos)

        if len(foods) > 0:
            foodDistance = self.SortDistances(newPos, foods)
            nearFood = foodDistance.pop()
            features["nearestFood"] = self.getMazeDistance((newPos), nearFood)
        features.divideAll(10.0)
        return features

    def getWeights(self, gameState, action):
        return {'nearestFood': -5, 'Capsule': 0,
                'ghost': -10, 'eatGhost': 1.0, 'ghostScared': 0.1, 'stop': -5, 'eatMoreFood': 1, 'goHome': 10, 'eatP': 10}

    def getSuccessor(self, gameState, action):
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != util.nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def reachTheEnd(self, depth):
        if depth == self.depth:
            return True
        else:
            return False

    def noMoreOpponent(self, gameState, opponent):
        if opponent not in self.getOpponents(gameState):
            return True
        else:
            return False

    def checkTheNumberOfGhost(self, gameState):
        if self.count == 0:
            return True
        else:
            return False

    def getEscape(self,gameState):
        ghostDistance = 100000000
        ghostIndexForMin=None
        minNode = -10000000000
        selectAction = []
        TheBegining = gameState.getLegalActions(self.index)
        for action in TheBegining:
            actionState = gameState.generateSuccessor(self.index, action)
            newGhostIndex = self.getOpponents(gameState)
            for index in newGhostIndex:
                if gameState.getAgentState(index).isPacman == False:
                    if gameState.getAgentState(index).getPosition() != None:
                        if self.getMazeDistance(gameState.getAgentState(index).getPosition(),
                                                gameState.getAgentPosition(self.index)) < ghostDistance:
                            ghostDistance = self.getMazeDistance(gameState.getAgentState(index).getPosition(),
                                                                 gameState.getAgentPosition(self.index))
                            ghostIndexForMin = index
            NodeValue = self.getMin(actionState, 0, ghostIndexForMin)
            if NodeValue > minNode:
                minNode = NodeValue
                selectAction = action
        return selectAction

    def getMax(self, gameState, depth=0):
        ghostDistance=100000000
        ghostIndexForMin=None
        if self.reachTheEnd(depth) == True :
            return self.EvaluationFunctionforEscape(gameState)

        maxNode = -10000000000
        pacmanAction = gameState.getLegalActions(self.index)

        for action in pacmanAction:
            newGhostIndex = self.getOpponents(gameState)
            for index in newGhostIndex:
                if gameState.getAgentState(index).isPacman == False:
                    if gameState.getAgentState(index).getPosition() != None:
                        if self.getMazeDistance(gameState.getAgentState(index).getPosition(), gameState.getAgentPosition(self.index))<ghostDistance :
                            ghostDistance=self.getMazeDistance(gameState.getAgentState(index).getPosition(), gameState.getAgentPosition(self.index))
                            ghostIndexForMin=index
                            gameState.getAgentState(index).scaredTimer = 0

            actionState = gameState.generateSuccessor(self.index, action)

            NodeValue = self.getMin(actionState, depth, ghostIndexForMin)

            if NodeValue > maxNode:
                maxNode = NodeValue
        return maxNode

    def getMin(self, gameState, depth=0, agentIndex=None):
        if self.reachTheEnd(depth) == True:
            return self.EvaluationFunctionforEscape(gameState)
        minNode = 10000000000
        if agentIndex!=None:
            ghostAction = gameState.getLegalActions(agentIndex)

            for action in ghostAction:

                actionState = gameState.generateSuccessor(agentIndex, action)
                NodeValue = self.getMax(actionState, depth + 1)

                if NodeValue < minNode:
                    minNode = NodeValue
        return minNode

    def EvaluationFunctionforEscape(self, GameState):

        CapsulesList=self.getCapsules(GameState)
        nearestFood = 10000000000
        newGhostposition = []
        scaretimers = []
        newPos = GameState.getAgentPosition(self.index)
        newFood = self.getFood(GameState).asList()
        newGhostIndex = self.getOpponents(GameState)

        Capsulescaneat=0
        nearestCapsules=1000000000
        obserPeriousCapsules=0
        if self.getCapsules(GameState) != []:
            for Capsules in CapsulesList:
                distance = self.getMazeDistance(newPos, Capsules)
                if distance < nearestCapsules:
                    nearestCapsules = distance
            Capsulescaneat=0.6*nearestCapsules
        perviousState = self.getPreviousObservation()
        if self.red:
            perviousCapsules = perviousState.getRedCapsules()
        else:
            perviousCapsules = perviousState.getBlueCapsules()
        perviousCapsulesLen = len(perviousCapsules)
        if perviousCapsulesLen > len(CapsulesList):
            obserPeriousCapsules = Capsulescaneat +1

        for food in newFood:
            distance =self.getMazeDistance(newPos, food)
            if distance < nearestFood:
                nearestFood = distance
        foodcaneat = 0.25 * nearestFood
        obserPeriousFood = 0
        perviousState = self.getPreviousObservation()
        if self.red:
            perviousFood = perviousState.getRedFood().asList()
        else:
            perviousFood = perviousState.getBlueFood().asList()
        perviousFoodLen = len(perviousFood)
        if perviousFoodLen > len(newFood):
            obserPeriousFood = foodcaneat+0.1
        goBack=0
        Ghost = []
        nearestGhost = 0
        for index in newGhostIndex:
            if GameState.getAgentState(index).isPacman == False:
                if GameState.getAgentState(index).getPosition() != None:
                    newGhostposition.append(GameState.getAgentState(index).getPosition())
                    scaretimers.append(GameState.getAgentState(index).scaredTimer)
        if newGhostposition != []:
            for ghostPosition in newGhostposition:
                Ghost.append(self.getMazeDistance(newPos, ghostPosition))
            nearestGhost = min(Ghost)

        if nearestGhost > 2:
            if GameState.getAgentState(self.index).numCarrying > 3:
                distance = self.SortDistances(perviousState.getAgentState(self.index).getPosition(), self.boundaryNew)
                sortest = distance.pop()
                dis = self.getMazeDistance(perviousState.getAgentState(self.index).getPosition(), sortest)
                distanceNew = self.SortDistances(newPos, self.boundaryNew)
                sort = distanceNew.pop()
                disst = self.getMazeDistance(newPos, sort)
                if disst < dis:
                    print("go back")
                    goBack = GameState.getAgentState(self.index).numCarrying+1
        else:
            distance = self.SortDistances(perviousState.getAgentState(self.index).getPosition(), self.boundaryNew)
            sortest = distance.pop()
            dis = self.getMazeDistance(perviousState.getAgentState(self.index).getPosition(), sortest)
            distanceNew = self.SortDistances(newPos, self.boundaryNew)
            sort = distanceNew.pop()
            disst = self.getMazeDistance(newPos, sort)
            if disst < dis:
                print("go back")
                goBack = GameState.getAgentState(self.index).numCarrying + 1
        return  - Capsulescaneat - foodcaneat  + obserPeriousFood + obserPeriousCapsules + goBack

    def getCloseGhostIndex(self, GameState):
        newPos = GameState.getAgentPosition(self.index)
        newGhostIndex = self.getOpponents(GameState)
        ghostDistance = 1000000
        indexGhost = None
        for index in newGhostIndex:
            if GameState.getAgentState(index).isPacman == False:
                if GameState.getAgentState(index).getPosition() != None:
                    if self.getMazeDistance(newPos, GameState.getAgentState(index).getPosition()) < ghostDistance:
                        indexGhost = index
        return indexGhost

    def boundaryPosition(self, gameState):
        ''''
    return a list of positions of boundary
    '''
        if self.red:
            boundary = int(gameState.getWalls().width / 2) - 1
        else:
            boundary = int(gameState.getWalls().width / 2)
        myboundary = []
        walls = gameState.getWalls().asList()
        currentPosition = gameState.getAgentPosition(self.index)
        for w in range(0, gameState.getWalls().height):
            if self.red:
                if not (boundary, w) in walls and not (boundary + 1, w) in walls:
                    if (boundary, w) != currentPosition:
                        myboundary.append((boundary, w))
            else:
                if not (boundary, w) in walls and not (boundary - 1, w) in walls:
                    if (boundary, w) != currentPosition:
                        myboundary.append((boundary, w))
        return myboundary



class goHome():
    def __init__(self, gameState, captureAgent):
        self.gameState = gameState
        self.myTeamAgent = captureAgent
        self.checkboundary()
        self.myBoundaryLocation = self.myBoundaryLocation()
        self.goal = self.getGoal()


    def checkboundary(self):
        if self.myTeamAgent.red == True:
            self.FoodICanEat = self.gameState.getRedFood()
            self.boundary = int(self.gameState.getWalls().width / 2) - 1
        else:
            self.FoodICanEat = self.gameState.getBlueFood()
            self.boundary = int(self.gameState.getWalls().width / 2)

    def myBoundaryLocation(self):
        myboundary = []
        walls = self.gameState.getWalls().asList()
        currentPosition = self.gameState.getAgentPosition(self.myTeamAgent.index)
        for w in range(0, self.gameState.getWalls().height):
            if self.myTeamAgent.red:
                if not (self.boundary, w) in walls and not (self.boundary + 1, w) in walls:
                    if (self.boundary, w) != currentPosition:
                        myboundary.append((self.boundary, w))
            else:
                if not (self.boundary, w) in walls and not (self.boundary - 1, w) in walls:
                    if (self.boundary, w) != currentPosition:
                        myboundary.append((self.boundary, w))
        return myboundary

    def getSuccessors(self, currentState):
        actions = currentState[0].getLegalActions(self.myTeamAgent.index)
        # print(currentState[0])
        pathLongth = self.myTeamAgent.getMazeDistance(currentState[0].getAgentPosition(self.myTeamAgent.index),
                                                      self.goal)
        betterSuccessors = []
        successors = [((currentState[0].generateSuccessor(
            self.myTeamAgent.index, action), pathLongth), action, 1) for action in actions]
        for better in successors:
            betterSuccessors.append(better)
        return betterSuccessors

    # def getGoal(self):
    #     distance=100000000
    #     myPosition= self.gameState.getAgentState(self.myTeamAgent.index).getPosition()
    #     boundary= self.myBoundaryLocation
    #     distination=None
    #     if boundary!=None:
    #         for boundaryLocation in boundary:
    #             if self.myTeamAgent.getMazeDistance(myPosition,boundaryLocation)<distance:
    #                 distance=self.myTeamAgent.getMazeDistance(myPosition,boundaryLocation)
    #                 distination= boundaryLocation
    #     return distination
    def SortDistances(self, startingPoint, destination):
        allDistance = util.PriorityQueue()
        for i in destination:
            allDistance.push(i, self.myTeamAgent.getMazeDistance(i, startingPoint))
        return allDistance

    def getGoal(self):
        allDistanceToBoundary = self.SortDistances(self.gameState.getAgentPosition(self.myTeamAgent.index),
                                                   self.myBoundaryLocation)
        nearestBoundary = allDistanceToBoundary.pop()
        return nearestBoundary

def nullHeuristic(state, question=None):
    return 0


def aStar(question, heuristic=nullHeuristic):
    store = util.PriorityQueue()
    visited = set()
    start = (question.gameState, question.myTeamAgent.getMazeDistance(
        question.goal, question.gameState.getAgentPosition(question.myTeamAgent.index)))
    h = heuristic(start, question)
    best = 0
    store.push((start, [], 0), 0 + h)
    while not store.isEmpty():
        next = store.pop()
        currentPosition = next[0][0].getAgentPosition(question.myTeamAgent.index)
        if currentPosition == question.goal:
            return next[1]
        if next[2] < best or next[0] not in visited:
            visited.add(next[0])
            if next[2] < best:
                best = next[2]
            new = question.getSuccessors(next[0])
            for nextPoint in new:
                if heuristic(nextPoint[0], question) < 0x7fffffff:
                    path = [] + next[1] + [nextPoint[1]]
                    cost = next[2] + nextPoint[2]
                    # print(cost)
                    store.update([nextPoint[0], path, cost], cost + heuristic(nextPoint[0], question))
    return set()


def defHeuristic(currentState, question=None):
    currentDistance = question.myTeamAgent.getMazeDistance(currentState[0].getAgentPosition(question.myTeamAgent.index),
                                                           question.goal)
    if currentDistance >= currentState[1]:
        return 0x7fffffff
    else:
        return 0


class defAgent(DummyAgent):
    lostFoodPoision = None
    theEnteryOfEnemy = False
    goToboundary = None
    boundaryPositionFirst = None
    countFordenfensive=None
    goHomeLocation = None
    def registerInitialState(self,gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.start = gameState.getAgentPosition(self.index)
        walls = gameState.getWalls().asList()
        self.max_X = max(wall[0] for wall in walls)
        self.max_Y = max(wall[1] for wall in walls)
        self.mid_X = self.max_X / 2
        self.mid_Y = self.max_Y / 2
        self.depth = 2
        self.finalScore = 0
        self.count = len(self.getOpponents(gameState))
        self.observationHistory = []
        self.boundaryNew = self.boundaryPosition(gameState)
        if self.red:
            self.boundary = int(gameState.getWalls().width / 2) - 1
        else:
            self.boundary = int(gameState.getWalls().width / 2)

    def chooseAction(self, gameState):
        # for opponent in self.getOpponents(gameState):
        #     if gameState.getAgentState(opponent).isPacman == True:
        if gameState.getAgentState(self.index).scaredTimer > 0:
            if gameState.getAgentState(self.index).scaredTimer <= 1:
                distance = 1000000000
                for opponent in self.getOpponents(gameState):
                    if gameState.getAgentState(opponent).isPacman == False:
                        if gameState.getAgentState(opponent).getPosition() is not None:
                            if gameState.getAgentState(opponent).scaredTimer == 0:
                                if self.getMazeDistance(gameState.getAgentPosition(self.index),
                                                        gameState.getAgentState(opponent).getPosition()) < distance:
                                    distance = self.getMazeDistance(gameState.getAgentPosition(self.index),
                                                                    gameState.getAgentState(opponent).getPosition())
                if distance < 3:
                    new = self.getEscape(gameState)
                    return new
                question = goHome(gameState, captureAgent=self)
                actions = aStar(question, heuristic=defHeuristic)
                if len(actions) != 0:
                    return actions[0]
                else:
                    return random.choice(gameState.getLegalActions(self.index))
            return self.chooseActionChange(gameState)

        question = defendLand(gameState, captureAgent=self)
        actions = aStar(question, heuristic=defHeuristic)
        if len(actions) != 0:
            return actions[0]
        else:
            return random.choice(gameState.getLegalActions(self.index))

    def evaluateForGoHome(self, gameState,action):
        evaGhost=0
        evaBoundary=0
        Pos = gameState.getAgentPosition(self.index)
        successor = self.getSuccessor(gameState, action)
        nextPos = successor.getAgentPosition(self.index)
        allDistanceToBoundary = self.SortDistances(gameState.getAgentPosition(self.index),
                                          self.boundaryNew)
        nearestBoundary = allDistanceToBoundary.pop()
        # if self.getMazeDistance(Pos,nearestBoundary)>self.getMazeDistance(nextPos,nearestBoundary):
        evaBoundary = -self.getMazeDistance(nextPos,nearestBoundary)
        print("boundary", evaBoundary)
        nearestGhost = None
        nearestGhostDistance = 1000000000
        for opponent in self.getOpponents(gameState):
            if gameState.getAgentState(opponent).isPacman == False:
                if gameState.getAgentState(opponent).getPosition() is not None:
                    if self.getMazeDistance(gameState.getAgentPosition(self.index),
                                            gameState.getAgentState(opponent).getPosition()) < nearestGhostDistance:
                        nearestGhostDistance = self.getMazeDistance(gameState.getAgentPosition(self.index),
                                                        gameState.getAgentState(opponent).getPosition())
                        nearestGhost = opponent
        if nearestGhost is not None:
            if gameState.getAgentState(nearestGhost).scaredTimer == 0:
                nearestGhostPos = gameState.getAgentPosition(nearestGhost)
                if self.getMazeDistance(Pos, nearestGhostPos) > self.getMazeDistance(nextPos, nearestGhostPos):
                    if self.getMazeDistance(Pos, nearestGhostPos)<=2:
                        evaGhost = self.getMazeDistance(nextPos, nearestGhostPos)*10
                        print("ghost",self.getMazeDistance(nextPos, nearestGhostPos)*10)
        print("final",evaGhost + evaBoundary)
        return evaGhost + evaBoundary

    def chooseActionChange(self, gameState):
        distance = 1000000000
        for opponent in self.getOpponents(gameState):
            if gameState.getAgentState(opponent).isPacman == False:
                if gameState.getAgentState(opponent).getPosition() is not None:
                   if gameState.getAgentState(opponent).scaredTimer == 0:
                        if self.getMazeDistance(gameState.getAgentPosition(self.index), gameState.getAgentState(opponent).getPosition()) < distance:
                            distance=self.getMazeDistance(gameState.getAgentPosition(self.index), gameState.getAgentState(opponent).getPosition())
        if distance < 3:
            new = self.getEscape(gameState)
            return new
        actions = gameState.getLegalActions(self.index)
        values = [self.evaluate(gameState, a) for a in actions]
        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]
        return random.choice(bestActions)

    def evaluate(self, gameState, action):
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

    # def evaluateGoHome(self, gameState, action):
    #     features = self.getGoHomeFeatures(gameState, action)
    #     weights = self.getWeights(gameState, action)
    #     return features * weights

    def SortDistances(self, startingPoint, destination):
        allDistance = util.PriorityQueue()
        for i in destination:
            allDistance.push(i, self.getMazeDistance(i, startingPoint))
        return allDistance

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        allWalls = gameState.getWalls()
        allFood = self.getFood(gameState)
        allcapsules = gameState.getCapsules()
        pos = gameState.getAgentState(self.index).getPosition()
        newPos = successor.getAgentState(self.index).getPosition()
        x, y = gameState.getAgentState(self.index).getPosition()
        nextX, nextY = successor.getAgentState(self.index).getPosition()
        foods = allFood.asList()
        if action == Directions.STOP:
            features['stop'] = 1
        opponentStates = [successor.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
        opponentGhost = []
        opponentInvaders = []
        for opponentState in opponentStates:
            if opponentState.getPosition() != None:
                if opponentState.isPacman:
                    opponentInvaders.append(opponentState)
                elif not opponentState.isPacman:
                    opponentGhost.append(opponentState)
        # for cap in allcapsules:
        #     if nextX == cap[0] and nextY == cap[1]:
        #         if successor.getAgentState(self.index).isPacman:
        #             features["Capsule"] = 1.0
        for opponent in opponentInvaders:
            gPos = opponent.getPosition()
            ghostNext = Actions.getLegalNeighbors(gPos,allWalls)
            if opponent.scaredTimer > 0:
                if (nextX, nextY) == gPos:
                    features["eatMoreFood"] += 2
                elif (nextX, nextY) in ghostNext:
                    features["ghostScared"] += 1
                elif successor.getAgentState(self.index).isPacman:
                    features["ghostScared"] = 0
                    features["ghost"] += 1

        ghostCount = 0
        near = 10000
        scarCount = 0
        for opp in self.getOpponents(gameState):
            if gameState.getAgentState(opp).isPacman == False:
                ghostCount = ghostCount + 1
            if gameState.getAgentState(opp).getPosition() is not None:
                di = self.getMazeDistance(gameState.getAgentState(opp).getPosition(), pos)
                if di < near:
                    near = di
            if gameState.getAgentState(opp).scaredTimer > 0 and gameState.getAgentState(opp).isPacman == False:
                scarCount = scarCount + 1
            if gameState.getAgentState(opp).isPacman:
                scarCount = scarCount + 1

        if near <= 3:
            if gameState.getAgentState(self.index).numCarrying >= 2:
                distance = self.SortDistances(pos, self.boundaryNew)
                sortest = distance.pop()
                dis = self.getMazeDistance(pos, sortest)
                distanceNew = self.SortDistances(newPos, self.boundaryNew)
                sort = distanceNew.pop()
                disst = self.getMazeDistance(newPos, sort)
                if disst < dis:
                    features["goHome"] = gameState.getAgentState(self.index).numCarrying
        elif ghostCount == 0 or scarCount==2:
            if gameState.getAgentState(self.index).numCarrying > 7:
                distance = self.SortDistances(pos, self.boundaryNew)
                sortest = distance.pop()
                dis = self.getMazeDistance(pos, sortest)
                distanceNew = self.SortDistances(newPos, self.boundaryNew)
                sort = distanceNew.pop()
                disst = self.getMazeDistance(newPos, sort)

                if disst < dis:
                    features["goHome"] = gameState.getAgentState(self.index).numCarrying
        else:
            if gameState.getAgentState(self.index).numCarrying > 4:
                distance = self.SortDistances(pos, self.boundaryNew)
                sortest = distance.pop()
                dis = self.getMazeDistance(pos, sortest)
                distanceNew = self.SortDistances(newPos, self.boundaryNew)
                sort = distanceNew.pop()
                disst = self.getMazeDistance(newPos, sort)

                if disst < dis:
                    features["goHome"] = gameState.getAgentState(self.index).numCarrying
        nearGhost = None
        # nearDistance = 1000
        # ghost = self.getOpponents(gameState)
        # for gh in ghost:
        #     if gameState.getAgentState(gh).isPacman:
        #         if gameState.getAgentState(gh).getPosition() is not None:
        #             if self.getMazeDistance(gameState.getAgentState(gh).getPosition(), newPos)< nearDistance:
        #                 nearDistance = self.getMazeDistance(gameState.getAgentState(gh).getPosition(), newPos)
        #                 nearGhost = gh
        if len(foods) > 0:
            foodDistance = self.SortDistances(newPos, foods)
            nearFood = foodDistance.pop()
            features["nearestFood"] = self.getMazeDistance((newPos), nearFood)
        features.divideAll(10.0)
        return features

    def getWeights(self, gameState, action):
        return {'nearestFood': -10, 'Capsule': 0,
                'ghost': -20, 'eatGhost': 1.0, 'ghostScared': 0.1, 'stop': -5, 'eatMoreFood': 1, 'goHome': 10}

    def getSuccessor(self, gameState, action):
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != util.nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def reachTheEnd(self, depth):
        if depth == self.depth:
            return True
        else:
            return False

    def noMoreOpponent(self, gameState, opponent):
        if opponent not in self.getOpponents(gameState):
            return True
        else:
            return False

    def checkTheNumberOfGhost(self, gameState):
        if self.count == 0:
            return True
        else:
            return False

    def getEscape(self,gameState):
        ghostDistance = 100000000
        ghostIndexForMin=None
        minNode = -10000000000
        selectAction = []
        TheBegining = gameState.getLegalActions(self.index)
        for action in TheBegining:
            actionState = gameState.generateSuccessor(self.index, action)
            newGhostIndex = self.getOpponents(gameState)
            for index in newGhostIndex:
                if gameState.getAgentState(index).isPacman == False:
                    if gameState.getAgentState(index).getPosition() != None:
                        if self.getMazeDistance(gameState.getAgentState(index).getPosition(),
                                                gameState.getAgentPosition(self.index)) < ghostDistance:
                            ghostDistance = self.getMazeDistance(gameState.getAgentState(index).getPosition(),
                                                                 gameState.getAgentPosition(self.index))
                            ghostIndexForMin = index
            NodeValue = self.getMin(actionState, 0, ghostIndexForMin)
            if NodeValue > minNode:
                minNode = NodeValue
                selectAction = action
        return selectAction

    def getMax(self, gameState, depth=0):
        ghostDistance=100000000
        ghostIndexForMin=None
        if self.reachTheEnd(depth) == True :
            return self.EvaluationFunctionforEscape(gameState)
        maxNode = -10000000000
        pacmanAction = gameState.getLegalActions(self.index)
        for action in pacmanAction:
            newGhostIndex = self.getOpponents(gameState)
            for index in newGhostIndex:
                if gameState.getAgentState(index).isPacman == False:
                    if gameState.getAgentState(index).getPosition() != None:
                        if self.getMazeDistance(gameState.getAgentState(index).getPosition(), gameState.getAgentPosition(self.index))<ghostDistance :
                            ghostDistance=self.getMazeDistance(gameState.getAgentState(index).getPosition(), gameState.getAgentPosition(self.index))
                            ghostIndexForMin=index
                            gameState.getAgentState(index).scaredTimer = 0
            actionState = gameState.generateSuccessor(self.index, action)
            NodeValue = self.getMin(actionState, depth, ghostIndexForMin)
            if NodeValue > maxNode:
                maxNode = NodeValue
        return maxNode

    def getMin(self, gameState, depth=0, agentIndex=None):
        if self.reachTheEnd(depth) == True:
            return self.EvaluationFunctionforEscape(gameState)
        minNode = 10000000000
        if agentIndex!=None:
            ghostAction = gameState.getLegalActions(agentIndex)
            for action in ghostAction:
                actionState = gameState.generateSuccessor(agentIndex, action)
                NodeValue = self.getMax(actionState, depth + 1)
                if NodeValue < minNode:
                    minNode = NodeValue
        return minNode

    def EvaluationFunctionforEscape(self, GameState):
        CapsulesList=self.getCapsules(GameState)
        nearestFood = 10000000000
        newGhostposition = []
        scaretimers = []
        newPos = GameState.getAgentPosition(self.index)
        newFood = self.getFood(GameState).asList()
        newGhostIndex = self.getOpponents(GameState)
        Capsulescaneat=0
        nearestCapsules=1000000000
        obserPeriousCapsules=0
        if self.getCapsules(GameState) != []:
            for Capsules in CapsulesList:
                distance = self.getMazeDistance(newPos, Capsules)
                if distance < nearestCapsules:
                    nearestCapsules = distance
            Capsulescaneat=0.6*nearestCapsules
        perviousState = self.getPreviousObservation()
        if self.red:
            perviousCapsules = perviousState.getRedCapsules()
        else:
            perviousCapsules = perviousState.getBlueCapsules()
        perviousCapsulesLen = len(perviousCapsules)
        if perviousCapsulesLen > len(CapsulesList):
            obserPeriousCapsules = Capsulescaneat +0.6
        for food in newFood:
            distance =self.getMazeDistance(newPos, food)
            if distance < nearestFood:
                nearestFood = distance
        foodcaneat = 0.25 * nearestFood
        obserPeriousFood = 0
        perviousState = self.getPreviousObservation()
        if self.red:
            perviousFood = perviousState.getRedFood().asList()
        else:
            perviousFood = perviousState.getBlueFood().asList()
        perviousFoodLen = len(perviousFood)
        if perviousFoodLen > len(newFood):
            obserPeriousFood = foodcaneat+0.1
        goBack=0
        Ghost = []
        nearestGhost = 0
        for index in newGhostIndex:
            if GameState.getAgentState(index).isPacman == False:
                if GameState.getAgentState(index).getPosition() != None:
                    newGhostposition.append(GameState.getAgentState(index).getPosition())
                    scaretimers.append(GameState.getAgentState(index).scaredTimer)
        if newGhostposition != []:
            for ghostPosition in newGhostposition:
                Ghost.append(self.getMazeDistance(newPos, ghostPosition))
            nearestGhost = min(Ghost)

        if nearestGhost > 2:
            if GameState.getAgentState(self.index).numCarrying > 2:
                distance = self.SortDistances(perviousState.getAgentState(self.index).getPosition(), self.boundaryNew)
                sortest = distance.pop()
                dis = self.getMazeDistance(perviousState.getAgentState(self.index).getPosition(), sortest)
                distanceNew = self.SortDistances(newPos, self.boundaryNew)
                sort = distanceNew.pop()
                disst = self.getMazeDistance(newPos, sort)
                if disst < dis:
                    goBack = GameState.getAgentState(self.index).numCarrying+1
        else:
            distance = self.SortDistances(perviousState.getAgentState(self.index).getPosition(), self.boundaryNew)
            sortest = distance.pop()
            dis = self.getMazeDistance(perviousState.getAgentState(self.index).getPosition(), sortest)
            distanceNew = self.SortDistances(newPos, self.boundaryNew)
            sort = distanceNew.pop()
            disst = self.getMazeDistance(newPos, sort)
            if disst < dis:
                goBack = GameState.getAgentState(self.index).numCarrying+1
        return  - Capsulescaneat - foodcaneat  + obserPeriousFood + obserPeriousCapsules + goBack

    def getCloseGhostIndex(self, GameState):
        newPos = GameState.getAgentPosition(self.index)
        newGhostIndex = self.getOpponents(GameState)
        ghostDistance = 1000000
        indexGhost = None
        for index in newGhostIndex:
            if GameState.getAgentState(index).isPacman == False:
                if GameState.getAgentState(index).getPosition() != None:
                    if self.getMazeDistance(newPos, GameState.getAgentState(index).getPosition()) < ghostDistance:
                        indexGhost = index
        return indexGhost

    def boundaryPosition(self, gameState):
        ''''
    return a list of positions of boundary
    '''
        if self.red:
            boundary = int(gameState.getWalls().width / 2) - 1
        else:
            boundary = int(gameState.getWalls().width / 2)
        myboundary = []
        walls = gameState.getWalls().asList()
        currentPosition = gameState.getAgentPosition(self.index)
        for w in range(0, gameState.getWalls().height):
            if self.red:
                if not (boundary, w) in walls and not (boundary + 1, w) in walls:
                    if (boundary, w) != currentPosition:
                        myboundary.append((boundary, w))
            else:
                if not (boundary, w) in walls and not (boundary - 1, w) in walls:
                    if (boundary, w) != currentPosition:
                        myboundary.append((boundary, w))
        return myboundary

class defendLand():
    def __init__(self, gameState, captureAgent):
        self.gameState = gameState
        self.myTeamAgent = captureAgent
        self.checkboundary()
        self.defBoundary = self.mydefLocation()
        self.myBoundaryLocation = self.myBoundaryLocation()
        self.opponentBoundaryLocation = self.opponentBoundaryLocation()
        self.goal = self.getGoal()

    def checkboundary(self):
        if self.myTeamAgent.red == True:
            self.FoodICanEat = self.gameState.getRedFood()
            self.boundary = int(self.gameState.getWalls().width / 2) - 1
        else:
            self.FoodICanEat = self.gameState.getBlueFood()
            self.boundary = int(self.gameState.getWalls().width / 2)

    def myBoundaryLocation(self):
        myboundary = []
        walls = self.gameState.getWalls().asList()
        currentPosition = self.gameState.getAgentPosition(self.myTeamAgent.index)
        for w in range(0, self.gameState.getWalls().height):
            if self.myTeamAgent.red:
                if not (self.boundary, w) in walls and not (self.boundary + 1, w) in walls:
                    if (self.boundary, w) != currentPosition:
                        myboundary.append((self.boundary, w))
            else:
                if not (self.boundary, w) in walls and not (self.boundary - 1, w) in walls:
                    if (self.boundary, w) != currentPosition:
                        myboundary.append((self.boundary, w))
        return myboundary

    def mydefLocation(self):
        myboundary = []
        walls = self.gameState.getWalls().asList()
        currentPosition = self.gameState.getAgentPosition(self.myTeamAgent.index)
        for w in range(0, self.gameState.getWalls().height):
            if self.myTeamAgent.red:
                if not (self.boundary, w) in walls and not (self.boundary + 1, w) in walls:
                    if (self.boundary, w) != currentPosition:
                        myboundary.append((self.boundary, w))
            else:
                if not (self.boundary, w) in walls and not (self.boundary - 1, w) in walls:
                    if (self.boundary, w) != currentPosition:
                        myboundary.append((self.boundary, w))
        if len(myboundary) > 4:
            myboundary.remove(myboundary[len(myboundary)-1])
            myboundary.remove(myboundary[len(myboundary)-1])
            myboundary.remove(myboundary[0])
            myboundary.remove(myboundary[0])
        elif 2 < len(myboundary) <= 4:
            myboundary.remove(myboundary[len(myboundary)-1])
            myboundary.remove(myboundary[0])
        elif len(myboundary) == 2:
            myboundary.remove(myboundary[0])
        return myboundary

    def opponentBoundaryLocation(self):
        opponentboundary = set()
        walls = self.gameState.getWalls().asList()
        currentPosition = self.gameState.getAgentPosition(self.myTeamAgent.index)
        if self.myTeamAgent.red:
            for w in range(0, self.gameState.getWalls().height):
                if not (self.boundary, w) in walls and not (self.boundary + 1, w) in walls:
                    if (self.boundary, w) != currentPosition:
                        opponentboundary.add((self.boundary + 1, w))
        else:
            for w in range(0, self.gameState.getWalls().height):
                if not (self.boundary, w) in walls and not (self.boundary - 1, w) in walls:
                    if (self.boundary, w) != currentPosition:
                        opponentboundary.add((self.boundary - 1, w))
        return opponentboundary

    def SortDistances(self, startingPoint, destination):
        allDistance = util.PriorityQueue()
        for i in destination:
            allDistance.push(i, self.myTeamAgent.getMazeDistance(i, startingPoint))
        return allDistance

    def getPositionBaseEnemy(self):
        closestFoodEntry = util.PriorityQueue()
        allBoundary = self.SortDistances(self.gameState.getAgentPosition(self.myTeamAgent.index),
                                         self.myBoundaryLocation)
        while not allBoundary.isEmpty():
            newLocation = allBoundary.pop()
            if self.myTeamAgent.getMazeDistance(self.gameState.getAgentPosition(self.myTeamAgent.index),
                                                newLocation) > 6:
                allFoodDistance = self.SortDistances(newLocation, self.FoodICanEat.asList())
                nearestFood = allFoodDistance.pop()
                nearestDistance = self.myTeamAgent.getMazeDistance(newLocation, nearestFood)
                closestFoodEntry.push(newLocation, nearestDistance)
        enemyPosition = closestFoodEntry.pop()
        if enemyPosition != None:
            return enemyPosition
        else:
            return random.choice(self.myBoundaryLocation)

    def getMissPoint(self):
        if self.myTeamAgent.getPreviousObservation() != None:
            lastFood = self.myTeamAgent.getFoodYouAreDefending(self.myTeamAgent.getPreviousObservation()).asList()
            NowFood = self.myTeamAgent.getFoodYouAreDefending(self.gameState).asList()
            if lastFood is not None:
                if len(lastFood) > len(NowFood):
                    Eatenfood = [food for food in lastFood if food not in NowFood]
                    if Eatenfood is not None:
                        return Eatenfood[0]
        return None

    def getSuccessors(self, currentState):
        actions = currentState[0].getLegalActions(self.myTeamAgent.index)
        print(currentState[0])
        pathLongth = self.myTeamAgent.getMazeDistance(currentState[0].getAgentPosition(self.myTeamAgent.index),
                                                      self.goal)
        betterSuccessors = []
        successors = [((currentState[0].generateSuccessor(
            self.myTeamAgent.index, action), pathLongth), action, 1) for action in actions]
        for better in successors:
            position = better[0][0]
            (x, y) = position.getAgentPosition(self.myTeamAgent.index)
            if self.myTeamAgent.red:
                if x <= self.boundary:
                    # if pathLongth <= currentState[1]:
                    betterSuccessors.append(better)
            else:
                if x >= self.boundary:
                    # if pathLongth <= currentState[1]:
                    betterSuccessors.append(better)
        return betterSuccessors

    def getGoal(self):
        # if self.gameState.getAgentState(self.myTeamAgent.index).isPacman:
        #     allDistanceToBoundary = self.SortDistances(self.gameState.getAgentPosition(self.myTeamAgent.index),
        #                                   self.myBoundaryLocation)
        #     nearestBoundary = allDistanceToBoundary.pop()
        #     return nearestBoundary
        lostFoodLocation = self.getMissPoint()
        StateAgent = self.gameState.getAgentState(self.myTeamAgent.index)
        dec = False
        if (StateAgent.scaredTimer > 0):
            goToboundary = self.SortDistances(self.gameState.getAgentPosition(self.myTeamAgent.index),
                                              self.myBoundaryLocation)
            if self.myTeamAgent.goToboundary == None:
                goToboundary = goToboundary.pop()
                self.myTeamAgent.goToboundary = goToboundary
            else:
                if self.myTeamAgent.goToboundary == self.gameState.getAgentPosition(self.myTeamAgent.index):
                    goToboundary = goToboundary.pop()
                    self.myTeamAgent.goToboundary = goToboundary
                else:
                    goToboundary = self.myTeamAgent.goToboundary
            return goToboundary

        if lostFoodLocation is not None:
            dec = True
        if dec:
            self.myTeamAgent.lostFoodPoision = lostFoodLocation
        for opponent in self.myTeamAgent.getOpponents(self.gameState) :
            if self.gameState.getAgentState(opponent).isPacman == True and self.gameState.getAgentPosition(
                    opponent) != None:
                self.myTeamAgent.theEnteryOfEnemy = True
                return self.gameState.getAgentPosition(opponent)
            elif self.gameState.getAgentState(opponent).isPacman == True and self.gameState.getAgentPosition(
                    opponent) == None:
                if self.myTeamAgent.lostFoodPoision is not None and self.myTeamAgent.theEnteryOfEnemy == True:
                    self.myTeamAgent.theEnteryOfEnemy = True
                    return self.myTeamAgent.lostFoodPoision
                else:
                    self.myTeamAgent.theEnteryOfEnemy = True
                    return self.getPositionBaseEnemy()
            else:
                self.myTeamAgent.theEnteryOfEnemy = False
        goToboundary = self.SortDistances(self.gameState.getAgentPosition(self.myTeamAgent.index),
                                          self.myBoundaryLocation)
        if self.myTeamAgent.goToboundary == None:
            goToboundary = goToboundary.pop()
            self.myTeamAgent.goToboundary = goToboundary
        else:
            if self.myTeamAgent.goToboundary == self.gameState.getAgentPosition(self.myTeamAgent.index):
                # goToboundary = goToboundary.pop()
                # self.myTeamAgent.goToboundary = goToboundary
                entryCanSee = False
                mayEntry = None
                for opponent in self.myTeamAgent.getOpponents(self.gameState):
                    if self.gameState.getAgentPosition(opponent) != None and self.gameState.getAgentState(
                        opponent).isPacman == False:
                        mayEntries = self.SortDistances(self.gameState.getAgentPosition(opponent), self.myBoundaryLocation)
                        mayEntry = mayEntries.pop()
                        entryCanSee =True
                if entryCanSee and mayEntry != None:
                    goToboundary = mayEntry
                    self.myTeamAgent.goToboundary = mayEntry
                elif len(self.defBoundary)!=1:
                    if self.myTeamAgent.countFordenfensive == None or self.myTeamAgent.countFordenfensive==len(self.defBoundary)-1:
                        goToboundary = self.defBoundary[0]
                        self.myTeamAgent.countFordenfensive=0
                    elif self.myTeamAgent.countFordenfensive==0:
                        goToboundary = self.defBoundary[int((len(self.defBoundary)-1)/2)]
                        self.myTeamAgent.countFordenfensive = int((len(self.defBoundary)-1)/2)
                    elif self.myTeamAgent.countFordenfensive==int((len(self.defBoundary)-1)/2):
                        goToboundary = self.defBoundary[(len(self.defBoundary))-1]
                        self.myTeamAgent.countFordenfensive=len(self.defBoundary)-1
                else:
                    goToboundary = self.defBoundary[0]
                self.myTeamAgent.goToboundary = goToboundary
            else:
                goToboundary = self.myTeamAgent.goToboundary
        return goToboundary