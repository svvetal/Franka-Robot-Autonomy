from pathlib import Path
from random import sample, seed
from re import A
import time
import pickle
import numpy as np
import RobotUtil as rt
import Franka
import time
import mujoco as mj
from mujoco import viewer

# Seed the random object
seed(10)

# Open the simulator model from the MJCF file
xml_filepath = str(Path(__file__).resolve().parent.parent / "franka_emika_panda" / "panda_with_hand_torque.xml")

np.random.seed(0)
deg_to_rad = np.pi / 180.0

# Initialize robot object
mybot = Franka.FrankArm()

# Initialize some variables related to the simulation
joint_counter = 0

# Initializing planner variables as global for access between planner and simulator
plan = []
interpolated_plan = []
plan_length = len(plan)
inc = 1

# Add obstacle descriptions into pointsObs and axesObs
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

envpoints, envaxes = rt.BlockDesc2Points(
    rt.rpyxyz2H([0, 0.0, 0.0], [0.45, 0, 0.25]), [0.5, 0.4, 0.5]
)
pointsObs.append(envpoints), axesObs.append(envaxes)

# define start and goal
deg_to_rad = np.pi / 180.0

# set the initial and goal joint configurations
qInit = [-np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, 0, np.pi - np.pi / 6, 0]
qGoal = [np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, 0, np.pi - np.pi / 6, 0]

# Initialize some data containers for the RRT planner
rrtVertices = []  # list of vertices
rrtEdges = []  # parent of each vertex

rrtVertices.append(qInit)
rrtEdges.append(0)

thresh = 0.1
FoundSolution = False
SolutionInterpolated = False

# RRT Configuration Parameters
STEP_SIZE = 0.30
GOAL_BIAS = 0.10
MAX_ITERATIONS = 3000
USE_PATH_SHORTENING = True
SHORTENING_ITERATIONS = 150


# Utility function to find the index of the nearset neighbor in an array of neighbors in prevPoints
def FindNearest(prevPoints, newPoint):
    D = np.array(
        [np.linalg.norm(np.array(point) - np.array(newPoint)) for point in prevPoints]
    )
    return D.argmin()


# Utility function for smooth linear interpolation of RRT plan, used by the controller
def naive_interpolation(plan):
    angle_resolution = 0.01
    global interpolated_plan
    global SolutionInterpolated
    interpolated_plan = np.empty((1, 7))
    np_plan = np.array(plan)
    interpolated_plan[0] = np_plan[0]

    for i in range(np_plan.shape[0] - 1):
        max_joint_val = np.max(np_plan[i + 1] - np_plan[i])
        number_of_steps = int(np.ceil(max_joint_val / angle_resolution))
        inc = (np_plan[i + 1] - np_plan[i]) / number_of_steps

        for j in range(1, number_of_steps + 1):
            step = np_plan[i] + j * inc
            interpolated_plan = np.append(interpolated_plan, step.reshape(1, 7), axis=0)

    SolutionInterpolated = True
    print("Plan has been interpolated successfully!")


# TODO: - Create RRT to find path to a goal configuration by completing the function below.
def RRTQuery():
    global FoundSolution
    global plan
    global rrtVertices
    global rrtEdges

    while len(rrtVertices) < 3000 and not FoundSolution:
        # TODO: - Implement RRT algorithm to find a path to the goal configuration
        # Use the global rrtVertices, rrtEdges, plan and FoundSolution variables in your algorithm

        # Random Sampling with Goal Bias
        if np.random.rand() < GOAL_BIAS:
            qrand = np.array(qGoal, dtype=float)
        else:
            qrand = np.array(mybot.SampleRobotConfig(), dtype=float)

            # Only plan joints 1 to 5
            # Keep joints 6 and 7 fixed
            qrand[5:] = qInit[5:]

        # Find nearest node
        nearest_idx = int(FindNearest(rrtVertices, qrand))
        qnear = np.array(rrtVertices[nearest_idx], dtype=float)

        # RRT-Connect - repeatedly extend towards the qrand
        while len(rrtVertices) < MAX_ITERATIONS:
            distance = np.linalg.norm(qrand[:5] - qnear[:5])
            if distance < 1e-10:
                break

            # Compute the new configuration
            alpha = min(1.0, STEP_SIZE / distance)
            qnew = qnear + alpha * (qrand - qnear)

            # Collision check along the entire edge
            if mybot.DetectCollisionEdge(qnear, qnew, pointsObs, axesObs):
                break

            # Add collision-free node to the tree
            rrtVertices.append(qnew.copy())
            rrtEdges.append(nearest_idx)

            nearest_idx = len(rrtVertices) - 1
            qnear = qnew

            # Check whether the goal can be reached
            goal_distance = np.linalg.norm(qnear[:5] - np.array(qGoal)[:5])

            if goal_distance <= STEP_SIZE:

                if goal_distance < 1e-10:
                    FoundSolution = True
                    break

                if not mybot.DetectCollisionEdge(qnear, qGoal, pointsObs, axesObs):
                    rrtVertices.append(np.array(qGoal, dtype=float))
                    rrtEdges.append(nearest_idx)

                    FoundSolution = True
                    break

            # Stop extending once the sample is reached
            if alpha >= 1.0:
                break

    ### if a solution was found
    if FoundSolution:
        # Extract path
        c = -1  # Assume last added vertex is at goal
        plan.insert(0, rrtVertices[c])

        while True:
            c = rrtEdges[c]
            plan.insert(0, rrtVertices[c])
            if c == 0:
                break

        # TODO - Path shortening
        # Calculate original path length
        original_length = sum(
            np.linalg.norm(np.array(plan[k + 1])[:5] - np.array(plan[k])[:5])
            for k in range(len(plan) - 1)
        )

        print(f"Original path: {len(plan)} waypoints")
        print(f"Original length: {original_length:.4f}")

        for i in range(SHORTENING_ITERATIONS):

            # Skip shortening if disabled
            if not USE_PATH_SHORTENING:
                break

            if len(plan) < 3:
                break

            # Select two random waypoints
            a, b = sorted(np.random.choice(len(plan), size=2, replace=False))

            # Ensure at least one waypoint lies between them
            if b <= a + 1:
                continue

            # Check whether a direct shortcut is collision-free
            if not mybot.DetectCollisionEdge(plan[a], plan[b], pointsObs, axesObs):

                # Remove intermediate waypoints
                plan = plan[: a + 1] + plan[b:]

        # Calculate final length
        final_length = sum(
            np.linalg.norm(np.array(plan[k + 1])[:5] - np.array(plan[k])[:5])
            for k in range(len(plan) - 1)
        )

        print(f"Final path: {len(plan)} waypoints")
        print(f"Final length: {final_length:.4f}")
        print(f"Path reduction: " f"{100*(1-final_length/original_length):.2f}%")

        for i, q in enumerate(plan):
            print("Plan step: ", i, "and joint: ", q)

        plan_length = len(plan)
        naive_interpolation(plan)
        return

    else:
        print("No solution found")


################################# YOU DO NOT NEED TO EDIT ANYTHING BELOW THIS ##############################
def position_control(model, data):
    global joint_counter
    global inc
    global plan
    global plan_length
    global interpolated_plan

    # Instantite a handle to the desired body on the robot
    body = data.body("hand")

    # Check if plan is available, if not go to the home position
    if FoundSolution == False or SolutionInterpolated == False:
        desired_joint_positions = np.array(qInit)

    else:

        # If a plan is available, cycle through poses
        plan_length = interpolated_plan.shape[0]

        if (
            np.linalg.norm(interpolated_plan[joint_counter] - data.qpos[:7]) < 0.01
            and joint_counter < plan_length
        ):
            joint_counter += inc

        desired_joint_positions = interpolated_plan[joint_counter]

        if joint_counter == plan_length - 1:
            inc = -1 * abs(inc)
            joint_counter -= 1
        if joint_counter == 0:
            inc = 1 * abs(inc)

    # Set the desired joint velocities
    desired_joint_velocities = np.array([0, 0, 0, 0, 0, 0, 0])

    # Desired gain on position error (K_p)
    Kp = np.eye(7, 7) * 300

    # Desired gain on velocity error (K_d)
    Kd = 50

    # Set the actuator control torques
    data.ctrl[:7] = (
        data.qfrc_bias[:7]
        + Kp @ (desired_joint_positions - data.qpos[:7])
        + Kd * (desired_joint_velocities - data.qvel[:7])
    )


if __name__ == "__main__":

    # Load the xml file here
    model = mj.MjModel.from_xml_path(xml_filepath)
    data = mj.MjData(model)

    # Set the simulation scene to the home configuration
    mj.mj_resetDataKeyframe(model, data, 0)

    # Set to the initial configuration
    data.qpos[:7] = np.array(qInit)
    mj.mj_forward(model, data)

    # Set the position controller callback
    mj.set_mjcb_control(position_control)

    # Compute the RRT solution
    RRTQuery()

    viewer.launch(model, data)