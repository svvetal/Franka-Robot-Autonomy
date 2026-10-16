# Portfolio card copy

**Project title:** Franka Robot Autonomy — Collision-Aware Motion Planning

**Short description:** Built joint-space motion-planning pipelines for a 7-DOF Franka Panda manipulator in MuJoCo, combining kinematic modeling, SAT-based collision detection, RRT exploration with collision-checked shortcutting, and PRM roadmap construction.

**Tech stack:** Python · NumPy · MuJoCo · Robotics Kinematics · Jacobian IK · SAT Collision Detection · RRT · PRM · Motion Planning

**Detailed view:** Implemented geometric collision models for Franka links and obstacles using oriented bounding boxes and the Separating Axis Theorem. Developed a single-tree RRT extension strategy with 10% goal bias and randomized waypoint shortcuts, alongside a 1,000-configuration PRM with collision-validated roadmap edges. Integrated joint-space interpolation and gravity-compensated PD torque control for simulated playback. Supplementary exploration includes weighted damped-least-squares inverse kinematics.

**What to avoid claiming:** Real-robot deployment; guaranteed continuous collision avoidance; full completed force/impedance controllers; quantitative speedups or success rates not present in the executable logs; novel authorship of course-provided starter code.
