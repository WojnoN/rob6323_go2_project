# ROB6323 Go2 Project — Isaac Lab

This repository is the starter code for the NYU Reinforcement Learning and Optimal Control project in which students train a Unitree Go2 walking policy in Isaac Lab starting from a minimal baseline and improve it via reward shaping and robustness strategies. Please read this README fully before starting and follow the exact workflow and naming rules below to ensure your runs integrate correctly with the cluster scripts and grading pipeline.

## Repository policy

- Fork this repository and do not change the repository name in your fork.  
- Your fork must be named rob6323_go2_project so cluster scripts and paths work without modification.

### Prerequisites

- **GitHub Account:** You must have a GitHub account to fork this repository and manage your code. If you do not have one, [sign up here](https://github.com/join).

### Links
1.  **Project Webpage:** [https://machines-in-motion.github.io/RL_class_go2_project/](https://machines-in-motion.github.io/RL_class_go2_project/)
2.  **Project Tutorial:** [https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md](https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md)

## Connect to Greene

- Connect to the NYU Greene HPC via SSH; if you are off-campus or not on NYU Wi‑Fi, you must connect through the NYU VPN before SSHing to Greene.  
- The official instructions include example SSH config snippets and commands for greene.hpc.nyu.edu and dtn.hpc.nyu.edu as well as VPN and gateway options: https://sites.google.com/nyu.edu/nyu-hpc/accessing-hpc?authuser=0#h.7t97br4zzvip.

## Clone in $HOME

After logging into Greene, `cd` into your home directory (`cd $HOME`). You must clone your fork into `$HOME` only (not scratch or archive). This ensures subsequent scripts and paths resolve correctly on the cluster. Since this is a private repository, you need to authenticate with GitHub. You have two options:

### Option A: Via VS Code (Recommended)
The easiest way to avoid managing keys manually is to configure **VS Code Remote SSH**. If set up correctly, VS Code forwards your local credentials to the cluster.
- Follow the [NYU HPC VS Code guide](https://sites.google.com/nyu.edu/nyu-hpc/training-support/general-hpc-topics/vs-code) to set up the connection.

> **Tip:** Once connected to Greene in VS Code, you can clone directly without using the terminal:
> 1. **Sign in to GitHub:** Click the "Accounts" icon (user profile picture) in the bottom-left sidebar. If you aren't signed in, click **"Sign in with GitHub"** and follow the browser prompts to authorize VS Code.
> 2. **Clone the Repo:** Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`), type **Git: Clone**, and select it.
> 3. **Select Destination:** When prompted, select your home directory (`/home/<netid>/`) as the clone location.
>
> For more details, see the [VS Code Version Control Documentation](https://code.visualstudio.com/docs/sourcecontrol/intro-to-git#_clone-a-repository-locally).

### Option B: Manual SSH Key Setup
If you prefer using a standard terminal, you must generate a unique SSH key on the Greene cluster and add it to your GitHub account:
1. **Generate a key:** Run the `ssh-keygen` command on Greene (follow the official [GitHub documentation on generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key)).
2. **Add the key to GitHub:** Copy the output of your public key (e.g., `cat ~/.ssh/id_ed25519.pub`) and add it to your account settings (follow the [GitHub documentation on adding a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)).

### Execute the Clone
Once authenticated, run the following commands. Replace `<your-git-ssh-url>` with the SSH URL of your fork (e.g., `git@github.com:YOUR_USERNAME/rob6323_go2_project.git`).
```
cd $HOME
git clone <your-git-ssh-url> rob6323_go2_project
```
*Note: You must ensure the target directory is named exactly `rob6323_go2_project`. This ensures subsequent scripts and paths resolve correctly on the cluster.*
## Install environment

- Enter the project directory and run the installer to set up required dependencies and cluster-side tooling.  
```
cd $HOME/rob6323_go2_project
./install.sh
```
Do not skip this step, as it configures the environment expected by the training and evaluation scripts. It will launch a job in burst to set up things and clone the IsaacLab repo inside your greene storage. You must wait until the job in burst is complete before launching your first training. To check the progress of the job, you can run `ssh burst "squeue -u $USER"`, and the job should disappear from there once it's completed. It takes around **30 minutes** to complete. 
You should see something similar to the screenshot below (captured from Greene):

![Example burst squeue output](docs/img/burst_squeue_example.png)

In this output, the **ST** (state) column indicates the job status:
- `PD` = pending in the queue (waiting for resources).
- `CF` = instance is being configured.
- `R`  = job is running.

On burst, it is common for an instance to fail to configure; in that case, the provided scripts automatically relaunch the job when this happens, so you usually only need to wait until the job finishes successfully and no longer appears in `squeue`.

## What to edit

- In this project you'll only have to modify the two files below, which define the Isaac Lab task and its configuration (including PPO hyperparameters).  
  - source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py  
  - source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env_cfg.py
PPO hyperparameters are defined in source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/agents/rsl_rl_ppo_cfg.py, but you shouldn't need to modify them.

## How to edit

- Option A (recommended): Use VS Code Remote SSH from your laptop to edit files on Greene; follow the NYU HPC VS Code guide and connect to a compute node as instructed (VPN required off‑campus) (https://sites.google.com/nyu.edu/nyu-hpc/training-support/general-hpc-topics/vs-code). If you set it correctly, it makes the login process easier, among other things, e.g., cloning a private repo.
- Option B: Edit directly on Greene using a terminal editor such as nano.  
```
nano source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py
```
- Option C: Develop locally on your machine, push to your fork, then pull changes on Greene within your $HOME/rob6323_go2_project clone.

> **Tip:** Don't forget to regularly push your work to github

## Launch training

- From $HOME/rob6323_go2_project on Greene, submit a training job via the provided script.  
```
cd "$HOME/rob6323_go2_project"
./train.sh
```
- Check job status with SLURM using squeue on the burst head node as shown below.  
```
ssh burst "squeue -u $USER"
```
Be aware that jobs can be canceled and requeued by the scheduler or underlying provider policies when higher-priority work preempts your resources, which is normal behavior on shared clusters using preemptible partitions.

## Where to find results

- When a job completes, logs are written under logs in your project clone on Greene in the structure logs/[job_id]/rsl_rl/go2_flat_direct/[date_time]/.  
- Inside each run directory you will find a TensorBoard events file (events.out.tfevents...), neural network checkpoints (model_[epoch].pt), YAML files with the exact PPO and environment parameters, and a rollout video under videos/play/ that showcases the trained policy.  

## Download logs to your computer

Use `rsync` to copy results from the cluster to your local machine. It is faster and can resume interrupted transfers. Run this on your machine (NOT on Greene):

```
rsync -avzP -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null' <netid>@dtn.hpc.nyu.edu:/home/<netid>/rob6323_go2_project/logs ./
```

*Explanation of flags:*
- `-a`: Archive mode (preserves permissions, times, and recursive).
- `-v`: Verbose output.
- `-z`: Compresses data during transfer (faster over network).
- `-P`: Shows progress bar and allows resuming partial transfers.

## Visualize with TensorBoard

You can inspect training metrics (reward curves, loss values, episode lengths) using TensorBoard. This requires installing it on your local machine.

1.  **Install TensorBoard:**
    On your local computer (do NOT run this on Greene), install the package:
    ```
    pip install tensorboard
    ```

2.  **Launch the Server:**
    Navigate to the folder where you downloaded your logs and start the server:
    ```
    # Assuming you are in the directory containing the 'logs' folder
    tensorboard --logdir ./logs
    ```

3.  **View Metrics:**
    Open your browser to the URL shown (usually `http://localhost:6006/`).

## Debugging on Burst

Burst storage is accessible only from a job running on burst, not from the burst login node. The provided scripts do not automatically synchronize error logs back to your home directory on Greene. However, you will need access to these logs to debug failed jobs. These error logs differ from the logs in the previous section.

The suggested way to inspect these logs is via the Open OnDemand web interface:

1.  Navigate to [https://ood-burst-001.hpc.nyu.edu](https://ood-burst-001.hpc.nyu.edu).
2.  Select **Files** > **Home Directory** from the top menu.
3.  You will see a list of files, including your `.err` log files.
4.  Click on any `.err` file to view its content directly in the browser.

> **Important:** Do not modify anything inside the `rob6323_go2_project` folder on burst storage. This directory is managed by the job scripts, and manual changes may cause synchronization issues or job failures.

## Project scope reminder

- The assignment expects you to go beyond velocity tracking by adding principled reward terms (posture stabilization, foot clearance, slip minimization, smooth actions, contact and collision penalties), robustness via domain randomization, and clear benchmarking metrics for evaluation as described in the course guidelines.  
- Keep your repository organized, document your changes in the README, and ensure your scripts are reproducible, as these factors are part of grading alongside policy quality and the short demo video deliverable.

## Resources

- [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/ecosystem.html) — Everything you need to know about IsaacLab, and more!
- [Isaac Lab ANYmal C environment](https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/direct/anymal_c) — This targets ANYmal C (not Unitree Go2), so use it as a reference and adapt robot config, assets, and reward to Go2.
- [DMO (IsaacGym) Go2 walking project page](https://machines-in-motion.github.io/DMO/) • [Go2 walking environment used by the authors](https://github.com/Jogima-cyber/IsaacGymEnvs/blob/e351da69e05e0433e746cef0537b50924fd9fdbf/isaacgymenvs/tasks/go2_terrain.py) • [Config file used by the authors](https://github.com/Jogima-cyber/IsaacGymEnvs/blob/e351da69e05e0433e746cef0537b50924fd9fdbf/isaacgymenvs/cfg/task/Go2Terrain.yaml) — Look at the function `compute_reward_CaT` (beware that some reward terms have a weight of 0 and thus are deactivated, check weights in the config file); this implementation includes strong reward shaping, domain randomization, and training disturbances for robust sim‑to‑real, but it is written for legacy IsaacGym and the challenge is to re-implement it in Isaac Lab.
- **API References**:
    - [ArticulationData (`robot.data`)](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.assets.html#isaaclab.assets.ArticulationData) — Contains `root_pos_w`, `joint_pos`, `projected_gravity_b`, etc.
    - [ContactSensorData (`_contact_sensor.data`)](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sensors.html#isaaclab.sensors.ContactSensorData) — Contains `net_forces_w` (contact forces).

---
Students should only edit README.md below this line.

# Changes Made Table of Contents
The changes made will be split up into Three sections.
1. Provided by Tutorial
2. Rewards
3. Additions outside of tutorial

## Provided by Tutorial
This section will not go deep into reproduction steps due to their inclusion in the tutorial.MD until step 5.
1. Action Rewards/Penalties (Action Smoothing)

By introducing a buffer to keep track of the action sequences, we can compare an action with it's previous two actions so that great disparities, which signify great change, will have a greater penalty then actions with smaller changes.
 
2. Low-Level PD Controller 

Implementation of a PD Controller allows for greater fine tuning and control over all the joints by calculating the torques ourselves, and thus being able to adjust each aspect of it.

3. Constraints as Terminations - Height

Constraints as terminations allow for a much faster training process by terminating runs that have reached a failure condition. In the case of the tutorial, that signifies when the body of the bot drops below a certain threshold.

4. Raibert Hueristic (Gait):

In order to work on creating foot movement in the shape of a gait, the Raibert Heuristic sets up a reward system/function that allows for the promotion certain feet movement patterns. As a result, we are able to promote the bot to move with a certain gait.


5. Reward Refinement

Four rewards get introduced in this step:
 1. Orientation Reward
 2. Linear velocity on z axis
 3. Joint Velocities
 4. Angular velocity on x and y axis

 The implementation is put into two sections
 1. Reward setup within the Rob6323_go_2_env_cfg.py which is more of a penalty setup then rewards due to the negative calues.
```python
# Lines 52-55
orient_reward_scale = -5.0
lin_vel_z_reward_scale = -0.02
dof_vel_reward_scale = -0.0001
ang_vel_xy_reward_scale = -0.001
```

2. Then the per-episode implementation in Rob6323_go_2.py
```python
# Lines 63-75 - Adding the values to the logging
 self._episode_sums = {
            key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
            for key in [
                "track_lin_vel_xy_exp",
                "track_ang_vel_z_exp",
                "rew_action_rate",
                "raibert_heuristic",
                "orient",    # <<<< Additions
                "lin_vel_z", # <<<<
                "dof_vel",   # <<<<
                "ang_vel_xy" # <<<<
            ]
        }

...

#Lines 163 to 179 - Comments along with Reward Implementation
        # 1. Penalize non-vertical orientation (projected gravity on XY plane)
        # Hint: We want the robot to stay upright, so gravity should only project onto Z.
        # Calculate the sum of squares of the X and Y components of projected_gravity_b.
        projected_gravity_xy = self.robot.data.projected_gravity_b[:, :2]
        rew_orient = torch.sum(torch.square(projected_gravity_xy), dim=1) # Square the back two components of projected gravity

        # 2. Penalize vertical velocity (z-component of base linear velocity)
        # Hint: Square the Z component of the base linear velocity.
        rew_lin_vel_z = torch.square(self.robot.data.root_lin_vel_b[:, 2]) # Squaring the 3rd component of the root linear velocity

        # 3. Penalize high joint velocities
        # Hint: Sum the squares of all joint velocities.
        rew_dof_vel = torch.sum(torch.square(self.robot.data.joint_vel), dim=1) # Squaring all joint velocities

        # 4. Penalize angular velocity in XY plane (roll/pitch)
        # Hint: Sum the squares of the X and Y components of the base angular velocity.
        rew_ang_vel_xy = torch.sum(torch.square(self.robot.data.root_ang_vel_b[:, :2]), dim=1) # Summing the squares of the back two values of angular velocity

...

# Lines 188 to 197 - Implementation into the rewards array
 rewards = {
            "track_lin_vel_xy_exp": lin_vel_error_mapped * self.cfg.lin_vel_reward_scale,
            "track_ang_vel_z_exp": yaw_rate_error_mapped * self.cfg.yaw_rate_reward_scale,
            "rew_action_rate": rew_action_rate * self.cfg.action_rate_reward_scale,
            "raibert_heuristic": rew_raibert_heuristic * self.cfg.raibert_heuristic_reward_scale,
            "orient": rew_orient * self.cfg.orient_reward_scale,                # <<<< New Additions
            "lin_vel_z": rew_lin_vel_z * self.cfg.lin_vel_z_reward_scale,       # <<<<
            "dof_vel": rew_dof_vel * self.cfg.dof_vel_reward_scale,             # <<<<
            "ang_vel_xy": rew_ang_vel_xy * self.cfg.ang_vel_xy_reward_scale,    # <<<<
        }
```



 
6. Advanced Foot Interactions
In this step we implement rewards which help implement a smooth gait, foot clearance and contact forces. Foot clearance will allow for a reward or penalty to be administered at a certain step in the gait. The contact forces will allow for a reward or penalty to be applied if contact is being made at a certain time.
```python
    # Lines 52-56 - Setting up the feet IDS
        self._feet_ids = []
        for name in foot_names:
            id_list, _ = self.robot.find_bodies(name)
            self._feet_ids.append(id_list[0])

        # Lines 58-62  - Setting up the arrays for the foot contact sensors
        self._feet_ids_sensor = []
        for name in foot_names:
            id_list, _ = self._contact_sensor.find_bodies(name)
            self._feet_ids_sensor.append(id_list[0])

    ...

    # Lines 198-199 - Adding the rewards via functions
        rew_feet_clearance = self._reward_feet_clearance()
        rew_tracking_contacts_shaped_force = self._reward_tracking_contacts_shaped_force()


    # Lines 201-212 Adding to the rewards list
    rewards = {
            "track_lin_vel_xy_exp": lin_vel_error_mapped * self.cfg.lin_vel_reward_scale,
            "track_ang_vel_z_exp": yaw_rate_error_mapped * self.cfg.yaw_rate_reward_scale,
            "rew_action_rate": rew_action_rate * self.cfg.action_rate_reward_scale,
            "raibert_heuristic": rew_raibert_heuristic * self.cfg.raibert_heuristic_reward_scale,
            "orient": rew_orient * self.cfg.orient_reward_scale,
            "lin_vel_z": rew_lin_vel_z * self.cfg.lin_vel_z_reward_scale,
            "dof_vel": rew_dof_vel * self.cfg.dof_vel_reward_scale,
            "ang_vel_xy": rew_ang_vel_xy * self.cfg.ang_vel_xy_reward_scale,
            "feet_clearance": rew_feet_clearance * self.cfg.feet_clearence_reward_scale, # <<<
            "tracking_contacts_shaped_force": rew_tracking_contacts_shaped_force * self.cfg.tracking_contacts_shaped_force_reward_scale, # <<<<
        }

    ...


```

## Final Reward Values
This section will go over the final list of rewards at the end of it all with a small reasoning to the value.

```Python
# Termination Conditions
    base_height_min = 0.20  # Termination height, ends the run if the body of the robot drops below. Speeds up training

    # PD Control gains
    Kp = 20.0
    Kd = 0.5
    torque_limits = 100.0

    # Reward Scales
    raibert_heuristic_reward_scale = -10.0             # Penalize out of sync legs to help promote a smoother gait
    feet_clearence_reward_scale = -60.0                # Penalize legs being in the wrong position during a step
    tracking_contacts_shaped_force_reward_scale = 40.0 # Reward proper contact or lack there of as needed

    # Additional reward scales - Scales that get applied frequently and thus need a lower value.
    orient_reward_scale = -5.0        # Penalize improper body orientation to reinforce stability
    lin_vel_z_reward_scale = -0.02    # Penalize incorrect movements on the z axis to reinforce proper direction
    dof_vel_reward_scale = -0.0001    # Penalize torque usage above the limits of the robot to create realistic movements
    ang_vel_xy_reward_scale = -0.001  # Penalize incorrect rotations to maintain a stable body.

    action_rate_reward_scale = -0.1 # Penalize actions that are too large to prevent rapid joint acceleration and promote smooth movements
    lin_vel_reward_scale = 1.0      # Reward movement in the right direction
    yaw_rate_reward_scale = 0.5     # Reward facing the correct direction.
```


## Additions Outside of Tutorial
### 1. Friction
In order to introduce torque frictions into the simulator, no need will be needed to change in the config.
All changes will take place in rob6323_go2_env.py

#### Intiializing Friction for episodes:
The friction can be increased or decreased based on changing the number at the end, creating a random number between 0 and that upper bound.
```python
# Within the __init__ function, add the following

        # Friction epsilons
        self.eps_f = torch.rand(1, device=self.device)*2.5
        self.eps_mu = torch.rand(1, device=self.device)*0.3
```

```python
# Within _reset_idx, apply the same lines load a new value on reset

        # Reset Random
        self.eps_f = torch.rand(1, device=self.device)*2.5
        self.eps_mu = torch.rand(1, device=self.device)*0.3
```

#### Applying friction to the torque

When calculating the friction, you calculate the stictoin and viscous frictions separately, before adding them together to get the friction.

Afterwards, you add them together to get the total friction.

Lastly you subtract from the calculated torques.
```python
# Within the _apply_action function

# Friction calculation
        stiction = self.eps_f * torch.tanh(self.robot.data.joint_vel / 0.1)
        viscous = self.eps_mu * self.robot.data.joint_vel
        friction_torque = stiction + viscous

# ---------------  Leave untouched --------------------
        # PD Control Law
        torques = torch.clip(
            (
                self.Kp * (self.desired_joint_pos - self.robot.data.joint_pos) -
                self.Kd * self.robot.data.joint_vel
            ), -self.torque_limits, self.torque_limits
        )
# -----------------------------------------------------

        self.robot.set_joint_effort_target(torques-friction_torque) # <<<< Subtract friction_torque from the torque
```


### 2. Rough Terrain
(To run a training with rough terrain in  this fork, switch to SmallerTerrainTesting branch)
All additions for this section will be implemented within the rob6323_go2_env_cfg.py.

#### Imports
The following imports are additional to the existing ones:
```python
from isaaclab.sim import PhysxCfg
from isaaclab.terrains import TerrainGeneratorCfg
from isaaclab.terrains.height_field import HfRandomUniformTerrainCfg
```


#### Setting up the new terrain.
Most notes will be taken directly from the [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.terrains.html#isaaclab.terrains.TerrainGeneratorCfg)x

Below the simulator function, add the following:
```python
    # random-uniform terrain generator (height-field)
    RANDOM_UNIFORM_TERRAIN_CFG = TerrainGeneratorCfg(
        curriculum=False,                 # random (not row-based curriculum)
        difficulty_range=(0.0, 1.0),      # difficulty sampled U(low, high) when curriculum=False
        size=(8.0, 8.0),                  # sub-terrain size in meters 
        num_rows=10,                      # Number of rows of sub-terrains to generate.
        num_cols=20,                      # Number of columns of sub-terrains to generate.
        horizontal_scale=0.1,             # The discretization of the terrain along the x and y axes (in m).
        vertical_scale=0.005,             # The discretization of the terrain along the z axis (in m).
        slope_threshold=0.75,             # The slope threshold above which surfaces are made vertical.
        sub_terrains={
            "hf_random_uniform": HfRandomUniformTerrainCfg(
                proportion=1.0,            # always choose this terrain type 
                noise_range=(0.0, 0.06),   # min/max height noise (m) 
                noise_step=0.01,           # height quantization step (m) 
                # downsampled_scale=0.2,   # optional: sample on coarser grid then interpolate
            )
        },
    )
```

Make the following changes to the existing TerrainImporterConfig
```python
    # terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",                        # <<<< Change
        terrain_generator=RANDOM_UNIFORM_TERRAIN_CFG,    # <<<< Added
        max_init_Terrain_level= 2,                       # <<<< Added
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )
```

#### Notes for Changes
Changes aren't necessary in the other file since there are already important aspects included necessary for traversing rough terrain.

The maximum height of the bumps/edges is at .06 currently (noise range variable upper bound) while the gait checks look for the robot to lift its feet above .07, thus clearing existing obstacles.

As for orientation, there are rewards and penalties put in place to maintain a level body for the robot as well.