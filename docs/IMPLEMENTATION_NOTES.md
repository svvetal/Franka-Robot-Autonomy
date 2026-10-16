# Implementation notes

## Collision geometry

`motion_planning/Franka.py` defines a Franka 7-DOF articulated chain and a set of 12 oriented cuboid collision volumes. `CompCollisionBlockPoints()` composes each box's local transform with its parent joint frame. `motion_planning/RobotUtil.py` evaluates SAT overlap across the three face-normal axes of each cuboid plus nine edge-cross axes, skipping near-zero axes. Face touching counts as a collision at the implemented numerical tolerance.

**Limits:** Cuboids approximate rather than exactly represent mesh collision geometry. `DetectCollisionEdge()` samples only five interpolation points along a queried edge, so an undetected collision between samples is possible. The checker focuses on the modeled robot-environment overlaps; do not assume exhaustive robot self-collision coverage.

## RRT planning

`motion_planning/RRTQuery.py` performs randomized joint-space exploration biased toward the goal. As implemented, the planner varies joints 1–5 and retains joints 6–7 at the specified start configuration. Once a path is found, it tests up to 150 candidate shortcuts for sampled-edge collision. The script prints original and shortened waypoint counts and joint-space path lengths; no fixed performance metric is claimed, since results depend on random sampling.

The code currently labels the routine RRT-Connect-style: it repeatedly extends a single tree towards a sampled target. It is **not** the canonical two-tree bidirectional RRT-Connect algorithm.

## PRM planning

`motion_planning/PRMGenerator.py` rejection-samples 1,000 collision-free joint configurations. For each new sample, it connects reachable existing vertices within an L2 joint-space radius of 2.0 rad. The roadmap is serialized to a local pickle file. `motion_planning/PRMQuery.py` loads that graph and tries to connect the start and goal to roadmap neighbors before following its graph search and interpolating joint positions.

**Limits:** The supplied PRM query uses a simple goal-distance-guided graph exploration rather than a guaranteed shortest-path algorithm. It also uses finite edge sampling for collision validation. The generator may be slow due to repeated collision checks.

## Kinematics and control

The complete kinematics implementation appears in `motion_planning/Franka.py`, including geometric Jacobian computation, weighted damped least-squares IK updates, and FK pose construction. Trajectory playback uses joint-position error and velocity feedback with a gravity/bias compensation term. This is joint-space PD torque control, not a validated Cartesian impedance controller.

The separate `foundations/` files are **original course templates**, not completed solutions. Their gravity-compensation/position-control examples exist, but Cartesian force-control and impedance-control callbacks plus the introductory FK/IK template functions have not been implemented in these uploaded copies. These are retained for learning context only.

## Packaging changes vs. supplied source archives

- Collected the complete motion-planning scripts under `motion_planning/` and introductory examples under `foundations/`.
- Consolidated duplicate models to a single original licensed `franka_emika_panda/` tree (used the planning archive's version containing the hand and obstacle scene).
- Made model XML paths robust to the current working directory. Made generated PRM location relative to `motion_planning/`.
- Removed PDFs, simulator logs, `__pycache__`, and the precomputed PRM pickle.
- Added repository documentation, simple automated non-GUI tests, and source attribution.

The underlying user-provided algorithms, controller logic, and robot collision geometry constants have not been rewritten for this publication package.
