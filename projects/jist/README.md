# JIST

This is the implementation for [Joint Sampling and Trajectory Optimization Over Graphs for Online Motion Planning](https://arxiv.org/abs/2011.07171), Kalyan Vasudev Alwala and Mustafa Mukadam, IROS 2021.

JIST (JoInt Sampling and Trajectory optimization), is a unified approach that leverages the complementary strengths of sampling and optimization, and interleaves them both to tackle highly dynamic environments with long horizons that necessitate a fast online solution. See [project page](https://sites.google.com/view/jistplanner) for more details.


## Installation

- Install GTSAM and GPMP2 following the [steps here](../../README.md).
- Install Hydra 0.11: `pip install hydra-core==0.11`
- Install OpenCV 4.2.0.32: `pip install opencv-python==4.2.0.32`


## Usage

Run the following commands to visualize JIST in action.

- For Random Forest environment
  ```python
  python pointRobot_example.py problem.node_budget=100 problem.seed_val=1 dataset.obstacle_num=40 problem.sdf_side=65.0 dataset.vel_limit=1.5
  ```
- For Patrol Guard environment
  ```python
  python pointRobot_example_patrol_guard.py problem=example_patrol dataset.vel_limit=1.6 dataset.obstacle_num=2 problem.sdf_side=20
  ```


## Evaluation

To replicate Table 1 results from the paper, run the following commands.

- For Random Forest environment
  ```python
  python pointRobot_test.py dataset.vel_limit=1.5 dataset.obstacle_num=80 problem.node_budget=60 problem.sdf_side=60.0
  ```
- For Patrol Guard environment
  ```python
  python pointRobot_test_patrol_guard.py dataset.vel_limit=1.4 dataset.obstacle_num=2 problem.node_budget=100 problem.sdf_side=60.0 problem=example_patrol dataset=patrol_guard
  ```


## Citation

If you use JIST in an academic context, please cite following publication.

```latex
@inproceedings{alwala2021joint,
  title={Joint sampling and trajectory optimization over graphs for online motion planning},
  author={Alwala, Kalyan Vasudev and Mukadam, Mustafa},
  booktitle={International Conference on Intelligent Robots and Systems (IROS)},
  year={2021}
}
```


License
-----

JIST is released under the BSD license, reproduced in [LICENSE](../../LICENSE).
