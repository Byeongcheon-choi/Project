import mdp
import time

# The state space, defined as a list of tuples
# [{x, y, heading}]
print("Generating State space")
S = mdp.generateStateSpace(mdp.L, mdp.W, mdp.H)

print("Generating Action space")
# The action space, defined as a list of tuples
# [{change in x, change in y, change in heading}]
A = mdp.generateActionSpace()

# 2.3c
print("Plotting the trajectory for the initial policy")
initialPolicy = mdp.generateStartPolicy()
mdp.plotPolicy(initialPolicy, 0, plotTitle="Initial Policy Actions for a Heading of 0")
initialTrajectory = mdp.getTrajectory(mdp.generateStartPolicy(), (1,4,6), 0)
print(mdp.evaluateTrajectory(initialTrajectory, 0.9, False, numOfIterations=0))  # -0.656
mdp.plotTrajectory(initialTrajectory, "Trajectory of Robot for Initial Policy")

# Run policy iteration and plot trajectory. Record the runtime (Problem 2.3h and 2.3i)
beginTime = int(round(time.time() * 1000))
policy, values = mdp.policyIteration(0,0.9,S,A) #NOTE: Not clear if we should use lambda = 0.9
endTime = int(round(time.time() * 1000))
for h in range(1):
    mdp.plotPolicy(policy, h, values, plotTitle='Policy Actions for a Heading of 0 (Policy Iteration)')
trajectory = mdp.getTrajectory(policy, (1,4,6), 0)
print(mdp.evaluateTrajectory(trajectory, 0.9, False, numOfIterations=0))  # 4.783
mdp.plotTrajectory(trajectory, "Trajectory of Robot for Policy Iteration")
diffTime = endTime - beginTime
print(diffTime)  # 218697

# Run value iteration and plot trajectory. Record the runtime (Problem 2.4 b and 2.4c)
beginTime2 = int(round(time.time() * 1000))
print("Performing value iteration")
(policy2, values2) = mdp.valueIteration(0,0.9) #NOTE: Not clear if we should use lambda = 0.9
print("Value Iteration done")
for h in range(1):
    mdp.plotPolicy(policy2, h, values2, plotTitle='Policy Actions for a Heading of 0 (Value Iteration)')
endTime2 = int(round(time.time() * 1000))
diffTime2 = endTime2 - beginTime2
trajectory2 = mdp.getTrajectory(policy2, (1,4,6), 0)
print(mdp.evaluateTrajectory(trajectory2, 0.9, False, numOfIterations=0))  # 4.783
print(diffTime2)  # 381629
mdp.plotTrajectory(trajectory2, "Trajectory of Robot for Value Iteration")



# Run policy iteration and plot trajectory. (Problem 2.5a), p_e = 0.1
beginTime3 = int(round(time.time() * 1000))
(policy3, values3) = mdp.policyIteration(0.1,0.9,S,A) #NOTE: Not clear if we should use lambda = 0.9
endTime3 = int(round(time.time() * 1000))
for h in range(1):
    mdp.plotPolicy(policy3, h, values3, plotTitle='Policy Actions for a Heading of 0 (Policy Iteration, p_e=0.1)')
trajectory3 = mdp.getTrajectory(policy3, (1,4,6), 0.1)
print(mdp.evaluateTrajectory(trajectory3, 0.9, False, numOfIterations=0)) # 3.138
mdp.plotTrajectory(trajectory3, "Trajectory of Robot for Policy Iteration (p_e=0.1)")
diffTime3 = endTime3 - beginTime3
print(diffTime3)  # 424890


# Run value iteration and plot trajectory. (Problem 2.5a)
beginTime4 = int(round(time.time() * 1000))
print("Performing value iteration")
(policy4, values4) = mdp.valueIteration(0.1,0.9) #NOTE: Not clear if we should use lambda = 0.9
print("Value Iteration done")
for h in range(1):
    mdp.plotPolicy(policy4, h, values4, plotTitle='Policy Actions for a Heading of 0 (Value Iteration, p_e=0.1)')
endTime4 = int(round(time.time() * 1000))
diffTime4 = endTime4 - beginTime4
trajectory4 = mdp.getTrajectory(policy4, (1,4,6), 0.1)
print(mdp.evaluateTrajectory(trajectory4, 0.9, False, numOfIterations=0))  # 4.783
print(diffTime4)  # 575002
mdp.plotTrajectory(trajectory4, "Trajectory of Robot for Value Iteration (p_e=0.1)")


# Run policy iteration and plot trajectory. (Problem 2.5b)
# pe = 0%
beginTime5 = int(round(time.time() * 1000))
(policy5, values5) = mdp.policyIteration(0,0.9,S,A, is2_5B=True) #NOTE: Not clear if we should use lambda = 0.9
endTime5 = int(round(time.time() * 1000))
for h in range(1):
    mdp.plotPolicy(policy5, h, values5, plotTitle='Policy Actions for a Heading of 0 (Policy Iteration, p_e=0, 2.5b)', is2_5B=True)
trajectory5 = mdp.getTrajectory(policy5, (1,4,6), 0)
print(mdp.evaluateTrajectory(trajectory5, 0.9, True, numOfIterations=0)) # 4.783
mdp.plotTrajectory(trajectory5, "Trajectory of Robot for Policy Iteration (p_e=0, 2.5b)", is2_5B=True)
diffTime5 = endTime5 - beginTime5
print(diffTime5)  # 204888


# Run value iteration and plot trajectory. (Problem 2.5b)
# pe = 0%
beginTime6 = int(round(time.time() * 1000))
print("Performing value iteration")
(policy6, values6) = mdp.valueIteration(0,0.9, is2_5B=True) #NOTE: Not clear if we should use lambda = 0.9
print("Value Iteration done")
for h in range(1):
    mdp.plotPolicy(policy6, h, values6, plotTitle='Policy Actions for a Heading of 0 (Value Iteration, p_e=0, 2.5b)', is2_5B=True)
endTime6 = int(round(time.time() * 1000))
diffTime6 = endTime6 - beginTime6
trajectory6 = mdp.getTrajectory(policy6, (1,4,6), 0)
print(mdp.evaluateTrajectory(trajectory6, 0.9, True, numOfIterations=0))  # 4.783
print(diffTime6)  # 496027
mdp.plotTrajectory(trajectory6, "Trajectory of Robot for Value Iteration (p_e=0, 2.5b)", is2_5B=True)


# Run policy iteration and plot trajectory. (Problem 2.5b)
# pe = 25%
beginTime7 = int(round(time.time() * 1000))
(policy7, values7) = mdp.policyIteration(0.25,0.9,S,A, is2_5B=True) #NOTE: Not clear if we should use lambda = 0.9
endTime7 = int(round(time.time() * 1000))
for h in range(1):
    mdp.plotPolicy(policy7, h, values7, plotTitle='Policy Actions for a Heading of 0 (Policy Iteration, p_e=0.25, 2.5b)', is2_5B=True)
trajectory7 = mdp.getTrajectory(policy7, (1,4,6), 0.25)
print(mdp.evaluateTrajectory(trajectory7, 0.9, True, numOfIterations=0))  # 0.0
mdp.plotTrajectory(trajectory7, "Trajectory of Robot for Policy Iteration (p_e=0.25, 2.5b)", is2_5B=True)
diffTime7 = endTime7 - beginTime7
print(diffTime7)  # 464329


# Run value iteration and plot trajectory. (Problem 2.5b)
# pe = 25%
beginTime8 = int(round(time.time() * 1000))
print("Performing value iteration")
(policy8, values8) = mdp.valueIteration(0.25,0.9, is2_5B=True) #NOTE: Not clear if we should use lambda = 0.9
print("Value Iteration done")
for h in range(1):
    mdp.plotPolicy(policy8, h, values8, plotTitle='Policy Actions for a Heading of 0 (Value Iteration, p_e=0.25, 2.5b)', is2_5B=True)
endTime8 = int(round(time.time() * 1000))
diffTime8 = endTime8 - beginTime8
trajectory8 = mdp.getTrajectory(policy8, (1,4,6), 0.25)
print(mdp.evaluateTrajectory(trajectory8, 0.9, True, numOfIterations=0))  # 0.0
print(diffTime8)  # 652400
mdp.plotTrajectory(trajectory8, "Trajectory of Robot for Value Iteration (p_e=0.25, 2.5b)", is2_5B=True)