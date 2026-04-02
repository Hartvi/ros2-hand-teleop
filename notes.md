gazebo control
- `panda_controllers.yml` - list all the joints and controllers that we want to control
- in launch file - list all the necessary controllers from `panda_controllers.yml` - this is probably a one to one correspondence with the controllers listed in all our controller yml files
- in our urdf we have to add a `ros2_control` block for each joint that we want to control
- gazebo doesn't handle mimic joints, so we have to work with that



Calculate diff transform:
I have a robotic arm with two gripper fingers. The gripper fingers are both controlled using one parameter $q_F$. The kinematic chain from the base of the robot to the end effector on which both of the gripper fingers are attached is common for both fingers and is controlled by parameters $\vec{q}$. The total parameter vector is $Q = [\vec{q}, q_F]$. I will receive from a ROS2 node the input parameters as the following:
```
hand_points[4] # position of finger tip #1
hand_points[8] # position of finger tip #2
URDF
current joint positions [q, q_F]
```
The outputs should be the $\Delta q$.

I will want to use KDL and create a kinematic tree. I will most likely have to adjust positions of the two kinematic chains independently.

if distance between L & index 4 is smaller than distance between L & index 8 (otherwise flip 4 & 8):

$$
\min_{\Delta q} ||f_{kL}([\vec{q}, q_F] + \Delta q) - \mathrm{hand\_points[4]}||^2 + ||f_{kR}([\vec{q}, q_F] + \Delta q) - \mathrm{hand\_points[8]}||^2  + q^TAq
$$
where A is a diagonal matrix, q_F is the value of the gripper finger joint coordinate, \vec{q} is the rest of the kinematic chain.

iterate using newton, gradient descent or LM

---

## Architecture: topic vs service vs action

| Aspect | Topics (pub/sub) | Services (req/res) | Actions |
|--------|-------------------|---------------------|---------|
| Latency | Lowest — fire-and-forget | Higher — blocks caller until response | Highest |
| Throughput | Streaming at camera fps | One call at a time | One call + feedback stream |
| Matches existing arch? | Yes — trac_ik_node already uses this | No — would need caller rewrite | Overkill |

**Decision: topics.** The hand tracker publishes at camera frame rate (~30 Hz). We want an IK-style node that subscribes to `hand_points_corrected` (+ current `/joint_states`) and publishes `sensor_msgs/JointState` as fast as frames arrive. This is the same pattern as the existing `trac_ik_node`. No request-response overhead, no caller changes.

---

## Concrete derivation — Damped Least Squares (one-step LM per frame)

### Why Damped Least Squares (Levenberg-Marquardt)?

| Method | Pros | Cons |
|--------|------|------|
| Gradient descent | Simple | Slow convergence, needs step-size tuning |
| Newton | Quadratic convergence | Requires full Hessian — expensive, complex |
| Gauss-Newton | Uses $J^TJ$ instead of Hessian — natural for least-squares | Can diverge near singularities |
| **Damped Least Squares (LM)** | Gauss-Newton + damping $\lambda I$ — robust near singularities, fast convergence away from them, standard in IK solvers | One extra scalar to tune |

**Choice: Damped Least Squares.** For a real-time control loop at 30 Hz we take **one linearization step per frame** (no inner iterations). This is the resolved-rate IK approach.

### Setup

Panda kinematic tree (from the URDF):

```
panda_link0
 └─ panda_joint1 (revolute) ─ panda_link1
     └─ ... (joints 2-7) ...
         └─ panda_link8
             └─ panda_hand_joint (fixed) ─ panda_hand
                 ├─ panda_finger_joint1 (prismatic, axis +y) ─ panda_leftfinger   [chain L]
                 └─ panda_finger_joint2 (prismatic, axis –y) ─ panda_rightfinger  [chain R]
```

`panda_finger_joint2` mimics `panda_finger_joint1` with multiplier 1.0, so both are controlled by a single parameter $q_F$.

Independent parameter vector:

$$\theta = [q_1, q_2, \dots, q_7, q_F]^T \in \mathbb{R}^{8}$$

Two KDL chains extracted from the tree:
- **Chain L**: `panda_link0` → `panda_leftfinger` — 8 joints ($q_1 \dots q_7, q_{F1}$)
- **Chain R**: `panda_link0` → `panda_rightfinger` — 8 joints ($q_1 \dots q_7, q_{F2}$)

Since $q_{F2} = q_{F1} = q_F$, the mapping from $\theta$ to each chain's joint vector is the identity.

### Residual

Given target finger-tip positions $p_L, p_R \in \mathbb{R}^3$ from MediaPipe (after correction):

$$r(\theta) = \begin{bmatrix} f_{kL}(\theta) - p_L \\ f_{kR}(\theta) - p_R \end{bmatrix} \in \mathbb{R}^{6}$$

### Jacobian

KDL gives us the 6-row (twist) Jacobian for each chain. We keep only the top 3 rows (linear velocity part):

$$J_L = \frac{\partial f_{kL}}{\partial \theta}\bigg|_{\text{pos}} \in \mathbb{R}^{3 \times 8}, \qquad J_R = \frac{\partial f_{kR}}{\partial \theta}\bigg|_{\text{pos}} \in \mathbb{R}^{3 \times 8}$$

Stacked:

$$\bar{J} = \begin{bmatrix} J_L \\ J_R \end{bmatrix} \in \mathbb{R}^{6 \times 8}$$

### Linearised cost

$$C(\Delta\theta) = \|r + \bar{J}\,\Delta\theta\|^2 + \Delta\theta^T A\,\Delta\theta$$

where $A = \mathrm{diag}(a_1, \dots, a_8)$ penalises large motions (regularisation).

### Optimal step (closed-form)

Setting $\nabla_{\Delta\theta} C = 0$:

$$\boxed{(\bar{J}^T \bar{J} + A + \lambda I)\;\Delta\theta = -\bar{J}^T r}$$

This is an **8×8 symmetric positive-definite linear system** — trivially fast to solve with Eigen's `ldlt()`. One solve per frame, no iterations.

- $\lambda > 0$ is the LM damping factor (start with $\lambda \approx 0.01$)
- $A$ diagonal weights let you penalise arm joints less and gripper joint more (or vice versa)

### Damping strategy (simplified for real-time)

Since we are running one step per frame (not iterating to convergence), a fixed damping $\lambda$ is the simplest approach. A typical starting point:

$$\lambda = 0.01, \qquad A = \mathrm{diag}(\underbrace{0.001, \dots, 0.001}_{7}, \underbrace{0.01}_{q_F})$$

If the solution oscillates, increase $\lambda$. If it is sluggish, decrease it. These can be ROS parameters.

### Finger assignment heuristic

Before computing the residual, assign MediaPipe landmarks to the correct physical finger:

```
FK_L = f_kL(θ)   // current left fingertip position
FK_R = f_kR(θ)   // current right fingertip position

d_LL = ||FK_L - hand_points[4]||
d_LR = ||FK_L - hand_points[8]||

if d_LL <= d_LR:
    p_L = hand_points[4],  p_R = hand_points[8]   // normal assignment
else:
    p_L = hand_points[8],  p_R = hand_points[4]   // flipped
```

### Implementation steps (C++ node)

1. **Parse URDF → KDL Tree** using `kdl_parser`
2. **Extract two chains**: `tree.getChain("panda_link0", "panda_leftfinger", chain_L)` and same for `panda_rightfinger`
3. **Subscribe** to:
   - `hand_points_corrected` (`HandPoints`) — target fingertip positions
   - `/joint_states` (`sensor_msgs/JointState`) — current $\theta$
4. **Each callback**:
   a. Read current $\theta$ from latest joint state
   b. Compute FK for both chains → $f_{kL}(\theta)$, $f_{kR}(\theta)$
   c. Run finger assignment heuristic
   d. Compute Jacobians $J_L$, $J_R$ using `KDL::ChainJntToJacSolver`
   e. Extract top-3 rows (position part), stack into $\bar{J}$
   f. Form residual $r$
   g. Solve $(\bar{J}^T \bar{J} + A + \lambda I)\,\Delta\theta = -\bar{J}^T r$
   h. Apply: $\theta_{\text{new}} = \theta + \Delta\theta$, clamp to joint limits
   i. Publish $\theta_{\text{new}}$ on `/main_joint_states`
5. **Parameters** (all ROS params): `base_link`, `tip_link_left`, `tip_link_right`, `lambda`, `A_diag`, `robot_description`

### Dependencies

- `kdl_parser` — URDF → KDL tree
- `orocos_kdl` — FK solver, Jacobian solver
- `Eigen3` — 8×8 linear solve (`LDLT`)
- `rclcpp`, `sensor_msgs`, `hand_publisher_interfaces`