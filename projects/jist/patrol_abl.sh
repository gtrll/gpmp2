#!/usr/bin/env bash


# velocity ablation
python pointRobot_test_patrol_guard.py --multirun  hydra.sweep.dir=./ablation_studies/patrol_vel_abl \
dataset.vel_limit=1.2,1.4,1.6,1.8 dataset.obstacle_num=2 \
problem.node_budget=100 problem.sdf_side=60.0 problem=example_patrol dataset=patrol_guard

# Ostacle Number
python pointRobot_test_patrol_guard.py --multirun  hydra.sweep.dir=./ablation_studies/patrol_obs_abl \
dataset.vel_limit=1.4 dataset.obstacle_num=1,2 \
problem.node_budget=100 problem.sdf_side=60.0 problem=example_patrol dataset=patrol_guard

# Node budget
python pointRobot_test_patrol_guard.py --multirun  hydra.sweep.dir=./ablation_studies/patrol_node_abl \
dataset.vel_limit=1.4 dataset.obstacle_num=2 \
problem.node_budget=40,60,80,100,120 problem.sdf_side=60.0 problem=example_patrol dataset=patrol_guard

# Noise
python pointRobot_test_patrol_guard.py --multirun  hydra.sweep.dir=./ablation_studies/patrol_noise_abl \
dataset.vel_limit=1.4 dataset.obstacle_num=2 \
problem.node_budget=100 problem.sdf_side=60.0 problem.action_noise=0.0,0.05,0.1,0.15,0.20 \
problem=example_patrol dataset=patrol_guard


#common
python pointRobot_test_patrol_guard.py \
dataset.vel_limit=1.4 dataset.obstacle_num=2 \
problem.node_budget=100 problem.sdf_side=60.0 \
problem=example_patrol dataset=patrol_guard
