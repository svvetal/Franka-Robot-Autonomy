"""Show the HW2 Franka robot at the RRT goal configuration (static MuJoCo view).

Save this script inside HW2/Code/ and run: python3 spawn_franka_goal.py
The viewer displays the specified pose without advancing physics.
"""

from pathlib import Path
import time

import mujoco as mj
from mujoco import viewer
import numpy as np


# HW2 Section 2: joints 1..5 are planned; joints 6..7 remain fixed.
Q_GOAL = np.deg2rad([90.0, -90.0, -90.0, -90.0, 0.0, 150.0, 0.0])


def main():
    xml_path = (
        Path(__file__).resolve().parent
        / ".." / "franka_emika_panda" / "panda_with_hand_torque.xml"
    ).resolve()
    if not xml_path.is_file():
        raise FileNotFoundError(
            f"MuJoCo model not found: {xml_path}\n"
            "Place this script in HW2/Code/ next to RRTQuery.py."
        )

    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)

    # Restore the model keyframe, then override the seven arm joints.
    mj.mj_resetDataKeyframe(model, data, 0)
    data.qpos[:7] = Q_GOAL
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)

    print("Showing Franka at GOAL configuration")
    print("Joint angles (degrees):", np.rad2deg(data.qpos[:7]))
    print("Static view: close the MuJoCo viewer to exit.")

    # Passive viewer: no mj_step() means the robot stays exactly at Q_GOAL.
    with viewer.launch_passive(model, data) as window:
        window.sync()
        while window.is_running():
            time.sleep(0.05)


if __name__ == "__main__":
    main()
