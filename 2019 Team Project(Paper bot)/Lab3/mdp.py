#!python3

import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import random
import math
import copy

# Length and width
L = 6
W = 6
# Number of headings
H = 12

# Generates the state space, which is a list of tuples of all possible states
def generateStateSpace(length, width, headings):
    stateSpace = []
    for i in range(length):
        tempJ = []
        for j in range(width):
            tempH = []
            for k in range(headings):
                tempH.append((i, j, k))
            tempJ.append(tempH)
        stateSpace.append(tempJ)
    return stateSpace


# Generates the action space, which is a list of tuples of all state changes
def generateActionSpace():
    return ["forward", "reverse", "forward_cw", "forward_ccw", "reverse_cw", "reverse_ccw", "stay"]


# Generates the possible changes in the state for all the possible actions that could be taken
# s is a tuple
def possibleActionDeltas(s):
    possible_deltas = []
    # zero if facing upwards, 1 if facing right, 2 if facing down, 3 if facing left
    direction = math.floor((s[2] + 1) / 3) % 4
    if(direction == 0):
        possible_deltas.extend([(0, 1, 0), (0, -1, 0), (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1)])
    elif(direction == 1):
        possible_deltas.extend([(1, 0, 0), (-1, 0, 0), (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1)])
    elif(direction == 2):
        possible_deltas.extend([(0, -1, 0), (0, 1, 0), (0, -1, 1), (0, -1, -1), (0, 1, 1), (0, 1, -1)])
    elif(direction == 3):
        possible_deltas.extend([(-1, 0, 0), (1, 0, 0), (-1, 0, 1), (-1, 0, -1), (1, 0, 1), (1, 0, -1)])
    # check for edges cases
    possible_deltas[:] = [(0, delta[1], delta[2]) if(delta[0] == -1 and s[0] == 0) else delta for delta in possible_deltas]
    possible_deltas[:] = [(0, delta[1], delta[2]) if(delta[0] == 1 and s[0] == L-1) else delta for delta in possible_deltas]
    possible_deltas[:] = [(delta[0], 0, delta[2]) if(delta[1] == -1 and s[1] == 0) else delta for delta in possible_deltas]
    possible_deltas[:] = [(delta[0], 0, delta[2]) if(delta[1] == 1 and s[1] == W-1) else delta for delta in possible_deltas]
    # print("After check:" + str(possible_deltas))
    possible_deltas.append((0, 0, 0))
    # remove duplicates
    possible_deltas = list(set(possible_deltas))
    return possible_deltas


# Given a state and an action string, it converts it to an action delta
# s is a tuple
# a_string is the action, in string format
def actionToDelta(s, a_string):
    action = [0, 0, 0]
    if a_string == "stay":
        return tuple(action)
    # zero if facing upwards, 1 if facing right, 2 if facing down, 3 if facing left
    direction = math.floor((s[2] + 1) / 3) % 4
    if(direction == 0):
        if "forward" in a_string:
            action[1] = 1
        else:
            action[1] = -1
    elif(direction == 1):
        if "forward" in a_string:
            action[0] = 1
        else:
            action[0] = -1
    elif(direction == 2):
        if "forward" in a_string:
            action[1] = -1
        else:
            action[1] = 1
    else:
        if "forward" in a_string:
            action[0] = -1
        else:
            action[0] = 1

    if "cw" in a_string:
        action[2] = 1
    if "ccw" in a_string:
        action[2] = -1

    return tuple(action)


# Gives the probabilty of going from state s to s_prime, given action a was taken
# s, s_prime are tuples
# a_string is the action, in string format
# error_prob is the probability of making a turning error
def p_sa(error_prob, s, a_string, s_prime):
    # print("Trying it with " + str(error_prob) + ", " + str(s) + ", " + str(a_string) + ", " + str(s_prime) + ", ")
    a = actionToDelta(s, a_string)
    possible_deltas = possibleActionDeltas(s)
    if a not in possible_deltas:
        a = (0, 0, a[2])
    # print("Action Delta: " + str(a))
    if s_prime[0] >= L or s_prime[1] >= W or s_prime[2] >= H or s_prime[0] < 0 or s_prime[1] < 0 or s_prime[2] < 0:
        return 0
    if s[0] + a[0] >= L or s[0] + a[0] < 0 or s[1] + a[1] >= W or s[1] + a[1] < 0:
        return 0

    if(a == (0, 0, 0)):
        if s == s_prime:
            return 1
        else:
            return 0

    heading_delta = min(abs(s_prime[2] - (s[2] + a[2])), abs(s_prime[2] - (s[2] + a[2]) + H), abs(s_prime[2] - (s[2] + a[2]) - H))
    difference = (abs(s_prime[0] - (s[0] + a[0])), abs(s_prime[1] - (s[1] + a[1])), heading_delta)
    # print(difference)
    if difference[0] > 1 or difference[1] > 1 or difference[2] > 2:
        return 0
    if difference[2] == 2 and error_prob == 0: # can only rotate 2 if the error is non-zero
        return 0

    if all(val == 0 for val in difference):
        return 1 - (2 * error_prob)
    else:
        if error_prob == 0:
            return 0
        error_cw = (s[0], s[1], s[2] + 1)
        error_ccw = (s[0], s[1], s[2] - 1)
        return max(error_prob * p_sa(0, error_cw, a_string, s_prime), error_prob * p_sa(0, error_ccw, a_string, s_prime))


# Gets the next state based upon the probabilities given by p_sa
# error_prob is the probability of making a turning error
# s is a tuple
 # a_string is an action string
def getNextState(error_prob, s, a_string):
    if a_string == "stay":
        return s

    stateWithAnyError = s
    # probability of turn error is (2 * error_prob)
    if random.random() < (2 * error_prob):
        if random.random() > 0.5:
            stateWithAnyError = (s[0], s[1], (s[2] + 1) % 12)
        else:
            stateWithAnyError = (s[0], s[1], (s[2] - 1) % 12)
    actionDelta = actionToDelta(s, a_string)
    possible_deltas = possibleActionDeltas(s)
    if actionDelta not in possible_deltas:
        actionDelta = (0, 0, a[2])
    return (stateWithAnyError[0] + actionDelta[0], stateWithAnyError[1] + actionDelta[1], stateWithAnyError[2] + actionDelta[2])


# Returns the reward for being at state s
# s is a tuple
# is2_5B is a boolean
def getReward(s, is2_5B):
    if(s[0] == 0 or s[0] == L-1):
        return -100
    if(s[1] == 0 or s[1] == W-1):
        return -100
    if(s[0] == 4 and s[1] == 4):
        if(is2_5B is True and s[2] != 6):
            return 0
        else:
            return 1
    if(s[0] == 3 and (s[1] == 4 or s[1] == 3)):
        return -10
    return 0


# Creates the starting policy defined by problem 2.3
# The policy is a 2D list of L by W
def generateStartPolicy():
    goalX = 4
    goalY = 4
    startPolicy = [[[0 for h in range(H)] for y in range(W)] for x in range(L)]
    for i in range(L):
        for j in range(W):
            for k in range(H):
                if (i == goalX):
                    if (j == goalY):
                        startPolicy[i][j][k] = "stay"
                    elif (j < goalY):
                        if (k == 0):
                            startPolicy[i][j][k] = "forward"
                        elif(k == 6):
                            startPolicy[i][j][k] = "reverse"
                        elif(k >= 1 and k <= 3):
                            startPolicy[i][j][k] = "forward_ccw"
                        elif(k >= 4 and k <= 5):
                            startPolicy[i][j][k] = "reverse_cw"
                        elif(k >= 7 and k <= 8):
                            startPolicy[i][j][k] = "reverse_ccw"
                        elif(k >= 9 and k <= 11):
                            startPolicy[i][j][k] = "forward_cw"
                        else:
                            print("Error!")
                    elif (j > goalY):
                        if (k == 0):
                            startPolicy[i][j][k] = "reverse"
                        elif(k == 6):
                            startPolicy[i][j][k] = "forward"
                        elif(k >= 1 and k <= 3):
                            startPolicy[i][j][k] = "reverse_ccw"
                        elif(k >= 4 and k <= 5):
                            startPolicy[i][j][k] = "forward_cw"
                        elif(k >= 7 and k <= 8):
                            startPolicy[i][j][k] = "forward_ccw"
                        elif(k >= 9 and k <= 11):
                            startPolicy[i][j][k] = "reverse_cw"
                        else:
                            print("Error!")
                    else:
                        print("Error!")
                elif (i < goalX):
                    if (j == goalY):
                        if (k == 3):
                            startPolicy[i][j][k] = "forward"
                        elif (k == 9):
                            startPolicy[i][j][k] = "reverse"
                        elif (k >= 0 and k <= 2):
                            startPolicy[i][j][k] = "forward_cw"
                        elif (k >= 4 and k <= 6):
                            startPolicy[i][j][k] = "forward_ccw"
                        elif (k >= 7 and k <= 8):
                            startPolicy[i][j][k] = "reverse_cw"
                        elif (k >= 10 and k <= 11):
                            startPolicy[i][j][k] = "reverse_ccw"
                        else:
                            print("Error!")
                    elif (j < goalY):
                        if (k == 1 or k == 2 or k == 11):
                            if(k == 1 and j == goalY - 1):
                                startPolicy[i][j][k] = "forward_cw"
                            elif(k == 2 and i == goalX - 1):
                                startPolicy[i][j][k] = "forward_ccw"
                            elif(k == 11 and j == goalY - 1):
                                startPolicy[i][j][k] = "forward_ccw"
                            else:
                                startPolicy[i][j][k] = "forward"
                        elif (k == 7 or k == 8 or k == 5 or k == 10):
                            if(k == 5 and j == goalY - 1):
                                startPolicy[i][j][k] = "reverse_ccw"
                            elif(k == 7 and j == goalY - 1):
                                startPolicy[i][j][k] = "reverse_cw"
                            elif(k == 8 and i == goalX - 1):
                                startPolicy[i][j][k] = "reverse_ccw"
                            elif(k == 10 and i == goalX - 1):
                                startPolicy[i][j][k] = "reverse_cw"
                            else:
                                startPolicy[i][j][k] = "reverse"
                        elif (k == 3 or k == 4):
                            if(k == 4 and i == goalX - 1):
                                startPolicy[i][j][k] = "forward_cw"
                            else:
                                startPolicy[i][j][k] = "forward_ccw"
                        elif (k == 9):
                            startPolicy[i][j][k] = "reverse_ccw"
                        elif (k == 0):
                            startPolicy[i][j][k] = "forward_cw"
                        elif (k == 6):
                            startPolicy[i][j][k] = "reverse_cw"
                        else:
                            print("Error!")
                    elif (j > goalY):
                        if (k == 5):
                            startPolicy[i][j][k] = "forward_ccw"
                        elif (k == 8 or k == 10):
                            if(k == 8 and i == goalX - 1):
                                startPolicy[i][j][k] = "reverse_ccw"
                            elif(k == 10 and i == goalX - 1):
                                startPolicy[i][j][k] = "reverse_cw"
                            else:
                                startPolicy[i][j][k] = "reverse"
                        elif (k == 3 or k == 7 or k == 2):
                            startPolicy[i][j][k] = "forward_cw"
                        elif (k == 4 or k == 6):
                            if(k == 4 and i == goalX - 1):
                                startPolicy[i][j][k] = "forward_cw"
                            else:
                                startPolicy[i][j][k] = "forward_ccw"
                        elif (k == 0 or k == 11):
                            startPolicy[i][j][k] = "reverse_ccw"
                        elif (k == 1or k == 9):
                            startPolicy[i][j][k] = "reverse_cw"
                        else:
                            print("Error!")
                    else:
                        print("Error!")
                elif (i > goalX):
                    if (j == goalY):
                        if (k == 9):
                            startPolicy[i][j][k] = "forward"
                        elif (k == 3):
                            startPolicy[i][j][k] = "reverse"
                        elif (k >= 0 and k <= 2):
                            startPolicy[i][j][k] = "reverse_cw"
                        elif (k >= 4 and k <= 6):
                            startPolicy[i][j][k] = "reverse_ccw"
                        elif (k >= 7 and k <= 8):
                            startPolicy[i][j][k] = "forward_cw"
                        elif (k >= 10 and k <= 11):
                            startPolicy[i][j][k] = "forward_ccw"
                        else:
                            print("Error!")
                    elif (j < goalY):
                        if (k == 11):
                            if(k == 11 and j == goalY - 1):
                                startPolicy[i][j][k] = "forward_ccw"
                            else:
                                startPolicy[i][j][k] = "forward"
                        elif (k == 5):
                            if(k == 5 and j == goalY - 1):
                                startPolicy[i][j][k] = "reverse_ccw"
                            else:
                                startPolicy[i][j][k] = "reverse"
                        elif (k == 9 or k == 0 or k == 1 or k == 8):
                            startPolicy[i][j][k] = "forward_ccw"
                        elif (k == 10):
                            startPolicy[i][j][k] = "forward_cw"
                        elif (k == 3 or k == 4 or k == 7):
                            startPolicy[i][j][k] = "reverse_cw"
                        elif (k == 2 or k == 6):
                            startPolicy[i][j][k] = "reverse_ccw"
                        else:
                            print("Error!")
                    elif (j > goalY):
                        if (k == 8):
                            startPolicy[i][j][k] = "forward_ccw"
                        elif (k == 1):
                            startPolicy[i][j][k] = "reverse"
                        elif (k == 7 or k == 6 or k == 10):
                            startPolicy[i][j][k] = "forward_cw"
                        elif (k == 9 or k == 5):
                            startPolicy[i][j][k] = "forward_ccw"
                        elif (k == 0 or k == 4):
                            startPolicy[i][j][k] = "reverse_cw"
                        elif (k == 3 or k == 11 or k == 2):
                            startPolicy[i][j][k] = "reverse_ccw"
                        else:
                            print("Error!")
                    else:
                        print("Error!")
                else:
                    print("Error!")
    return startPolicy


# Creates a trajectory given a policy, initial state, and error probability
# The trajectory is a list of states
# policy is 3D list
# s_0 is a tuple
# error_prob is the probability of making a turning error
def getTrajectory(policy, s_0, error_prob):
    currentState = s_0
    trajectory = []
    action = policy[currentState[0]][currentState[1]][currentState[2]]
    counter = 0
    trajectory.append(currentState)
    while (action != "stay" and counter < 4):
        nextState = getNextState(error_prob, currentState, action)
        if(nextState == currentState):
            counter = counter + 1
        else:
            counter = 0
        trajectory.append(nextState)
        currentState = nextState
        action = policy[currentState[0]][currentState[1]][currentState[2]]
    if(action == "stay"):
        trajectory.append(trajectory[-1])
    return trajectory


# evaluates a given trajectory based upon the discount_factor
# note that numOfIterations is used for the recursive calculation and thus need not be supplied
def evaluateTrajectory(trajectory, discount_factor, is2_5B, numOfIterations=0):
    if not trajectory:
        return 0
    if len(trajectory) == 2 and trajectory[0] == trajectory[1]:
        # return the infinite value of being at that state, minus the amount of elapsed time
        summation = 0
        if numOfIterations > 0:
            for x in range(numOfIterations):
                summation = summation + (discount_factor ** x)
        temp = (1 / (1-discount_factor)) - summation  # Need to remove the amount before it settled at this spot
        return getReward(trajectory[0], is2_5B) * temp
    return evaluateTrajectory(trajectory[1:], discount_factor, is2_5B, numOfIterations+1) + ((discount_factor ** numOfIterations) * getReward(trajectory[0], is2_5B))


# Plots the given trajectory
def plotTrajectory(trajectory, plotTitle='', is2_5B=False):
    rewards = np.zeros((L, W))
    for x in range(L):
        for y in range(W):
            rewards[(x, y)] = getReward((x, y, 6), is2_5B)
    # rewards[0, :] = -100
    # rewards[5, :] = -100
    # rewards[:, 0] = -100
    # rewards[:, 5] = -100
    # rewards[(4, 4)] = 10
    # rewards[(3, 4)] = -10
    # rewards[(3, 3)] = -10
    # Transpose since it's graphed (row,col) instead of (x,y), like the getReward() function returns
    rewards = np.transpose(rewards)
    # Flip vertically since matplotlib plots with positive y being in direction of bottom of screen
    rewards = np.flipud(rewards)

    # create discrete colormap
    cmap = colors.ListedColormap(['tomato', 'goldenrod', 'white', 'green'])
    bounds = [-150, -90, -5, 0.5, 15]
    norm = colors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots()
    ax.imshow(rewards, interpolation='none', cmap=cmap, norm=norm, zorder=0)

    # draw gridlines
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    ax.set_xticks(np.arange(-0.5, L, 1))
    ax.set_yticks(np.arange(-0.5, W, 1))
    x_labels = range(L)
    y_labels = range(W)
    new_x_labels = []
    new_y_labels = []
    for label in x_labels:
        new_x_labels.append("              " + str(label))
    for label in y_labels:
        new_y_labels.append("\n\n\n\n" + str(W - 1 - label))
    ax.set_xticklabels(new_x_labels)
    ax.set_yticklabels(new_y_labels)

    headingStrings = [['' for y in range(W)] for x in range(L)]

    # draw the arrows
    for j in range(len(trajectory)):
        # Make sure we're not at the last spot in trajectory
        start = trajectory[j]
        if(j != len(trajectory) - 1):
            end = trajectory[j+1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dh = end[2] - start[2]
            heading_delta = min(abs(end[2] - start[2]), abs(end[2] - start[2] + H), abs(end[2] - start[2] - H))
            if(dh != heading_delta):
                if(end[2] == (start[2] + heading_delta) % 12):
                    dh = heading_delta
                else:
                    dh = -1 * heading_delta

            if(dx == 0 and dy == 0):
                if(dh == 0):
                    ax.add_artist(plt.Circle((start[0], W - 1 - start[1]), 0.1, color='black'))
                elif(dh < 0):
                    ax.add_artist(plt.Circle((start[0], W - 1 - start[1]), 0.1, color='teal'))
                else:
                    ax.add_artist(plt.Circle((start[0], W - 1 - start[1]), 0.1, color='magenta'))
            else:
                if(dh == 0):
                    plt.arrow(start[0], W - 1 - start[1], dx, -dy, head_width=0.1, length_includes_head=True, color='black')
                elif(dh < 0):
                    plt.arrow(start[0], W - 1 - start[1], dx, -dy, head_width=0.1, length_includes_head=True, color='teal')
                else:
                    plt.arrow(start[0], W - 1 - start[1], dx, -dy, head_width=0.1, length_includes_head=True, color='magenta')

        if(headingStrings[start[0]][start[1]] == ''):
            headingStrings[start[0]][start[1]] = str(start[2])
            # print("found empty")
        else:
            headingStrings[start[0]][start[1]] = headingStrings[start[0]][start[1]] + ", " + str(start[2])
            # print("Found one that was already full")

    for x in range(L):
        for y in range(W):
            hString = headingStrings[x][y]
            ax.annotate(hString, xy=(x - 0.45, W - 1 - y + 0.45))

    ax.set_title(plotTitle)
    plt.show()


# Plots the given trajectory
def plotPolicy(input_policy, h, values=[], plotTitle='', is2_5B=False):
    rewards = np.zeros((L, W))
    for x in range(L):
        for y in range(W):
            rewards[(x, y)] = getReward((x, y, 6), is2_5B)
    # Transpose since it's graphed (row,col) instead of (x,y), like the getReward() function returns
    rewards = np.transpose(rewards)
    # Flip vertically since matplotlib plots with positive y being in direction of bottom of screen
    rewards = np.flipud(rewards)

    # create discrete colormap
    cmap = colors.ListedColormap(['tomato', 'goldenrod', 'white', 'green'])
    bounds = [-150, -90, -5, 0.5, 15]
    norm = colors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots()
    ax.imshow(rewards, interpolation='none', cmap=cmap, norm=norm, zorder=0)

    # draw gridlines
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    ax.set_xticks(np.arange(-0.5, L, 1))
    ax.set_yticks(np.arange(-0.5, W, 1))
    x_labels = range(L)
    y_labels = range(W)
    new_x_labels = []
    new_y_labels = []
    for label in x_labels:
        new_x_labels.append("              " + str(label))
    for label in y_labels:
        new_y_labels.append("\n\n\n\n" + str(W - 1 - label))
    ax.set_xticklabels(new_x_labels)
    ax.set_yticklabels(new_y_labels)

    # draw the arrows/dots
    for x in range(L):
        for y in range(W):
            actionDelta = actionToDelta((x, y, h), input_policy[x][y][h])
            possible_deltas = possibleActionDeltas((x, y, h))
            # print("Policy action: " + str(input_policy[x][y][h]))
            # print("Action delta: " + str(actionDelta))
            # print("Possible deltas: " + str(possible_deltas))
            if actionDelta not in possible_deltas:
                actionDelta = (0, 0, actionDelta[2])
            start = (x, y, h)
            end = (x + actionDelta[0], y + actionDelta[1], h + actionDelta[2])
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            if(dx == 0 and dy == 0):
                if(actionDelta[2] == 0):
                    ax.add_artist(plt.Circle((start[0], W - 1 - start[1]), 0.1, color='black'))
                elif(actionDelta[2] < 0):
                    ax.add_artist(plt.Circle((start[0], W - 1 - start[1]), 0.1, color='teal'))
                else:
                    ax.add_artist(plt.Circle((start[0], W - 1 - start[1]), 0.1, color='magenta'))
            else:
                if(actionDelta[2] == 0):
                    plt.arrow(start[0], W - 1 - start[1], dx / 2.0, -dy / 2.0, head_width=0.1, length_includes_head=True, color='black')
                elif(actionDelta[2] < 0):
                    plt.arrow(start[0], W - 1 - start[1], dx / 2.0, -dy / 2.0, head_width=0.1, length_includes_head=True, color='teal')
                else:
                    plt.arrow(start[0], W - 1 - start[1], dx / 2.0, -dy / 2.0, head_width=0.1, length_includes_head=True, color='magenta')
            if values:
                ax.annotate("{0:.2f}".format(round(values[x][y][h], 2)), xy=(x - 0.45, W - 1 - y + 0.45), color='blue')

    if not plotTitle:
        ax.set_title("Policy Actions for a Heading of " + str(h))
    else:
        ax.set_title(plotTitle)
    plt.show()
    # for j in range(len(trajectory)):
    #     # Make sure we're not at the last spot in trajectory
    #     if(j != len(trajectory) - 1):
    #         start = trajectory[j]
    #         end = trajectory[j+1]
    #         dx = end[0] - start[0]
    #         dy = end[1] - start[1]
    #         # Since we can't graph the roations without it being cluttered, just ignore it
    #         if(dx == 0 and dy == 0):
    #             continue
    #         plt.arrow(start[0], W - 1 - start[1], dx, -dy, head_width=0.1, length_includes_head=True, color='black')
    

# Returns the evaluation of the given policy (value for each state)
# The evaluation is a 3D list
# The prevValue is the previous value function
def evaluatePolicy(policy,prevValue,error_prob,gamma,stateSpace, is2_5B=False):
    newValueFunc = []
    #iterate over the s states
    #print(stateSpace)
    for i in stateSpace:
        tempRows = []
        for j in i:
            tempHeaders = []
            for s in j:
                summation = 0
                #pi(s)
                policyOfs = policy[s[0]][s[1]][s[2]]
                
                # iterate over the s' states
                for iPrime in stateSpace:
                    for jPrime in iPrime:
                        for sPrime in jPrime:
                            t = p_sa(error_prob,s,policyOfs,sPrime)
                            temp = t * (getReward(s, is2_5B) + (gamma * prevValue[sPrime[0]][sPrime[1]][sPrime[2]]))
                            summation = summation + temp
                tempHeaders.append(summation)
            tempRows.append(tempHeaders)
        newValueFunc.append(tempRows)
    return newValueFunc


# Returns the optimal policy given a one-step lookahead on the Value (Problem 2.3f)
# The policy is a 3D list of L by W by H
def optimalPolicyUpdate(value,stateSpace,actionSpace, error_prob, gamma, is2_5B=False):
    newPolicy = [[['' for h in range(H)] for y in range(W)] for x in range(L)]
    #get the max value for each action in the action state:
    for i in stateSpace:
        for j in i:
            for s in j:
                optimalA = actionSpace[0]
                maxActionValue = float('-inf')
                for a in actionSpace:
                    summation = 0
                    # iterate over the s' states
                    #NOTE: You don't iterate over the s' state. You only need to use the action to get the next state from that action...
                    #The way we do this right now is ineffecient and could be improved in the future if run-time is an issue
                    #Instead of going over all of s', we could choose to go over only the s' that are actually possible (i.e. most of the P(s' | s,a) are zero right now
                    for iPrime in stateSpace:
                        for jPrime in iPrime:
                            for sPrime in jPrime:
                                t = p_sa(error_prob,s,a,sPrime)
                                if t == 0:  # if zero then we know that it's not possible to go here
                                    continue
                                temp = t * (getReward(s, is2_5B) + (gamma * value[sPrime[0]][sPrime[1]][sPrime[2]]))
                                summation = summation + temp
                    if (summation > maxActionValue):
                        maxActionValue = summation
                        optimalA = a
                        # print("summation > maxActionValue: " + str(summation) + " > " + str(maxActionValue))
                newPolicy[s[0]][s[1]][s[2]] = optimalA
    return newPolicy


# Run policy iteration (Problem 2.3g)
def policyIteration(error_prob,gamma,stateSpace,actionSpace, is2_5B=False):
    #initialize policy
    policy = generateStartPolicy()
    """policy = []
    for i in range(L):
        tempH = []
        for k in range(W):
            tempW = []
            for j in range(H):
                tempW.append(0)
            tempH.append(tempW)
        policy.append(tempH)"""


    prevPolicy = [] 
    value = [[[0 for h in range(H)] for y in range(W)] for x in range(L)]
    iterCount = 0
    print("getting to first policy iteration")
    while (prevPolicy != policy):#i.e. we continue to make changes to the policy
        print("copying previous policy")
        prevPolicy = copy.deepcopy(policy)
        print("evaluating first policy")
        value = evaluatePolicy(prevPolicy, value, error_prob, gamma, stateSpace, is2_5B=is2_5B)
        print("finished policy evaluation")
        policy = optimalPolicyUpdate(value,stateSpace,actionSpace, error_prob, gamma, is2_5B=is2_5B)
        # plotPolicy(policy, 0, value)
        print("finished optimal policy update")
        print("Iteration: %i", iterCount)
        iterCount = iterCount + 1    
    print("finished loop")
    return (policy, value)


# Run value iteration (Problem 2.4a)
def valueIteration(error_prob,gamma, is2_5B=False):
    #Arbitrary epsilon to set. It is the fraction of the value to stop at
    max_error = 1
    epsilon = ((1-gamma) / (2 * gamma)) * max_error

    # Initialize V(s) = 0
    value = [[[0 for h in range(H)] for y in range(W)] for x in range(L)]

    stateSpace = generateStateSpace(L, W, H)
    actionSpace = generateActionSpace()
    # Loop and then loop again
    """while (value ):
        for i in stateSpace:
            for k in i:
                for s in k:
                    newValue = getReward(s) + """
    newValue = [[[0 for h in range(H)] for y in range(W)] for x in range(L)]
    counter = 0
    while(True):
        for i in stateSpace:
            for j in i:
                for s in j:
                    maxActionValue = float('-inf')
                    for a in actionSpace:
                        summation = 0
                        for iPrime in stateSpace:
                            for jPrime in iPrime:
                                for sPrime in jPrime:
                                    t = p_sa(error_prob, s, a, sPrime)
                                    if t == 0:  # if zero then we know that it's not possible to go here
                                        continue
                                    temp = t * (getReward(s, is2_5B) + (gamma * value[sPrime[0]][sPrime[1]][sPrime[2]]))
                                    summation = summation + temp
                        if (summation > maxActionValue):
                            maxActionValue = summation
                    newValue[s[0]][s[1]][s[2]] = maxActionValue

        #Test to see if the Value has converged
        newValueFlattened = [newVal for x in newValue for y in x for newVal in y]
        valueFlattened = [val for x in value for y in x for val in y]
        # print(newValueFlattened)
        # print(valueFlattened)
        differences = [nV - v for (nV, v) in zip(newValueFlattened, valueFlattened)]
        # print(differences)
        # print(value)
        absDifferences = [abs(d) for d in differences]
        absValue = [abs(v) for v in valueFlattened]
        print("Went through iteration number: " + str(counter))
        counter = counter + 1
        # print("max(absDifferences) < epsilon * min(absValue): " + str(max(absDifferences)) + " < " + str(epsilon * (max(absValue))))
        # print((sum(absValue)/len(absValue)))
        hasConverged = True
        for j in range(len(absDifferences)):
            absDiff = absDifferences[j]
            absVal = absValue[j]
            # print("absDiff > epsilon * (absVal): " + str(absDiff) + " > " + str(epsilon * (absVal)))
            if(absDiff > epsilon * (absVal)):
                hasConverged = False
                break
        if hasConverged is True:
            value = copy.deepcopy(newValue)
            break
        else:
            value = copy.deepcopy(newValue)
        # if counter > 1:
        #     value = copy.deepcopy(newValue)
        #     break

    return (optimalPolicyUpdate(value, stateSpace, actionSpace, error_prob, gamma, is2_5B=is2_5B), value)
