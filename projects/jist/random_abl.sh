#!/usr/bin/env bash


# Ostacle Number
python pointRobot_test.py --multirun  hydra.sweep.dir=./ablation_studies/random_obs_abl \
dataset.vel_limit=1.5 dataset.obstacle_num=20,40,60,80,100 \
problem.node_budget=60 problem.sdf_side=60.0

# Node budget
python pointRobot_test.py --multirun  hydra.sweep.dir=./ablation_studies/random_node_abl \
dataset.vel_limit=1.5 dataset.obstacle_num=80 \
problem.node_budget=40,60,80,100,120 problem.sdf_side=60.0

# Noise
python pointRobot_test.py --multirun  hydra.sweep.dir=./ablation_studies/random_noise_abl \
dataset.vel_limit=1.5 dataset.obstacle_num=80 \
problem.node_budget=60 problem.sdf_side=60.0 problem.action_noise=0.0,0.05,0.1,0.15,0.20


# velocity ablation
python pointRobot_test.py --multirun  hydra.sweep.dir=./ablation_studies/random_vel_abl \
dataset.vel_limit=0.5,1.0,1.5,2.0,2.5 dataset.obstacle_num=80 \
problem.node_budget=60 problem.sdf_side=60.0

# common
python pointRobot_test.py \
dataset.vel_limit=1.5 dataset.obstacle_num=80 \
problem.node_budget=60 problem.sdf_side=60.0 
