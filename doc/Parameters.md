Parameters
=========================================
GPMP2 algorithms have a few parameters. Here we briefly explain their meanings and how to tune/set them.

Optimization Parameters
-----
All the trajectory optimization related parameters are stored in [TrajOptimizerSetting](../gpmp2/planner/TrajOptimizerSetting.h) class. 

```cpp
/// general setting of all trajectory optimizers, batch and incremental
struct TrajOptimizerSetting {

  /// optimization iteration types
  enum IterationType {GaussNewton, LM, Dogleg};

  /// trajectory settings
  size_t dof;                   // degree of freedom, must be given explicitly
  size_t total_step;            // number of steps (states) optimized the whole trajectory
  double total_time;            // time duration (second) of the whole trajectory
  gtsam::SharedNoiseModel conf_prior_model;    // prior constraint model for initial/end state
  gtsam::SharedNoiseModel vel_prior_model;     // prior constraint model for initial/end velocity

  /// obstacle cost settings
  double epsilon;               // eps of hinge loss function (see the paper)
  double cost_sigma;            // sigma of obstacle cost (see the paper)
  size_t obs_check_inter;       // number of point interpolated for obstacle cost,
                                // 0 means do not interpolate
  /// GP settings
  gtsam::SharedNoiseModel Qc_model;    // Qc for GP (see the paper)

  /// Optimization settings
  IterationType opt_type;       // optimizer type
  double rel_thresh;            // relative error decrease threshold for stopping optimization
  size_t max_iter;              // max iteration for stopping optimization
  ...
```

Here we give brief explanation on each parameter and how to tune/set them. 
Please refer to the paper for the exact meaning of some parameters. 

- **dof**: Active degrees of freedom of the robot. 
- **total_step**: Total number of states/waypoints for optimization. For most cases >=10 is OK. 
    - Make sure the product of *total_step* and *obs_check_inter*+1 (which means the number of states used to calculate obstacle cost) is about 50-100 or greater. 
    - For example, *total_step*=10 and *obs_check_inter*=4 is good (total 10x(4+1)=50 states checked), *total_step*=50 and *obs_check_inter*=0 is also good (total 50x(0+1)=50 states checked. This is slower, since no GP interpolation and more states optimized), but *total_step*=10 and *obs_check_inter*=0 (only 10x(0+1)=10 states checked) has too few states for obstacle cost and will fail.
    - If you have small obstacles in the environment which the robot fails to avoid, increase the number of states to be checked (increase either *total_step* or *obs_check_inter*).
- **obs_check_inter**: number of states are checked by GP interpolation, 0 means GP interpolation is not used. 
    - Check *total_step* for details about how many states need to be interpolated.
- **total_time**: Total runtime of the trajectory in seconds.
- **conf_prior_model** and **vel_prior_model**: pose/velocity prior model covariance. Leave default value 0.0001 in most cases.
- **epsilon_dist**: \epsilon for hinge loss function. For most robots 0.05m-0.2m is good, depends on robot size. 
    - \epsilon refers to the 'safety distance', if you want robot stay faraway from obstacles, use bigger \epsilon. In contrast, more compact environments that have smaller scale need smaller \epsilon. 
- **cost_sigma**: \sigma_obs for obstacle cost. For 2D datasets use 0.05-0.2, for 3D datasets use 0.005-0.02.
    - \sigma_obs controls the balance between 'smoothness' of the trajectory and 'obstacle avoidance'. Smaller \sigma_obs refers to less smooth trajectory, with lower probability of colliding with obstacles. Larger \sigma_obs refers to more smooth trajectory, but higher probability of colliding with obstacles. Tune this value for your own robot, this is the only parameter that needs to be tuned based on the robot.
- **Qc_model**: GP hyperparameter. If this is not known just use identity.
- **opt_type**: Optimization algorithm. *LMA* works fine in most cases. *GaussNewton* may also work for simple problems.
- **rel_thresh**: Stop optimization when relative error decrease is smaller than this threshold. Leave default value 1e-6 in most cases. If you would like make the optimization stopping earlier, and you are OK with sub-optimial solution, make this value larger, like 1e-3; 
- **max_iter**: Max number of iterations of optimization, default is 100. If you would like to stop the algorithm earlier, or you have a run time limit set this to a smaller value. Note that too few iterations may not allow convergence to the correct minimal value.
