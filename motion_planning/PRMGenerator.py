from pathlib import Path
import Franka
import numpy as np
import random
import pickle
import RobotUtil as rt
import time

random.seed(13)

# Initialize robot object
mybot = Franka.FrankArm()

# Create environment obstacles - # these are blocks in the environment/scene (not part of robot)
pointsObs = []
axesObs = []

envpoints, envaxes = rt.BlockDesc2Points(
    rt.rpyxyz2H([0, 0.0, 0.0], [0.1, 0, 1.0]), [1.3, 1.4, 0.1]
)
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(
    rt.rpyxyz2H([0, 0.0, 0.0], [0.1, -0.65, 0.475]), [1.3, 0.1, 0.95]
)
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(
    rt.rpyxyz2H([0, 0.0, 0.0], [0.1, 0.65, 0.475]), [1.3, 0.1, 0.95]
)
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(
    rt.rpyxyz2H([0, 0.0, 0.0], [-0.5, 0, 0.475]), [0.1, 1.2, 0.95]
)
pointsObs.append(envpoints), axesObs.append(envaxes)

# Central block ahead of the robot
envpoints, envaxes = rt.BlockDesc2Points(
    rt.rpyxyz2H([0, 0.0, 0.0], [0.45, 0, 0.25]), [0.5, 0.4, 0.5]
)
pointsObs.append(envpoints), axesObs.append(envaxes)

prmVertices = []  # list of vertices
prmEdges = []  # adjacency list (undirected graph)
start = time.time()


# TODO: Create PRM - generate collision-free vertices
# TODO: Fill in the following function using prmVertices and prmEdges to store the graph.
# The code at the end saves the graph into a python pickle file.
def PRMGenerator():
    global prmVertices
    global prmEdges
    global pointsObs
    global axesObs

    pointsObs = np.array(pointsObs)
    axesObs = np.array(axesObs)

    while len(prmVertices) < 1000:
        # Sample random robot configuration
        q_new = mybot.SampleRobotConfig()

        # Keep joints 6 and 7 fixed
        q_new[5] = 5 * np.pi / 6
        q_new[6] = 0.0

        # Reject Configs in collision
        if mybot.DetectCollision(q_new, pointsObs, axesObs):
            continue

        # Add collision free vertices
        new_index = len(prmVertices)
        prmVertices.append(q_new)
        prmEdges.append([])

        # Find neighbours within 2 radians
        for i in range(new_index):
            distance = np.linalg.norm(np.array(q_new) - np.array(prmVertices[i]))

            if distance < 2.0:
                # Check Collision along edge
                if not mybot.DetectCollisionEdge(
                    q_new, prmVertices[i], pointsObs, axesObs
                ):
                    # Add undirected edge
                    prmEdges[new_index].append(i)
                    prmEdges[i].append(new_index)
        print(len(prmVertices))

    # Save the PRM such that it can be run by PRMQuery.py
    f = open(Path(__file__).resolve().parent / "myPRM.p", "wb")
    pickle.dump(prmVertices, f)
    pickle.dump(prmEdges, f)
    pickle.dump(pointsObs, f)
    pickle.dump(axesObs, f)
    f.close()


if __name__ == "__main__":

    # Call the PRM Generator function and generate a graph
    PRMGenerator()

    print("\n", "Vertices: ", len(prmVertices), ", Time Taken: ", time.time() - start)
