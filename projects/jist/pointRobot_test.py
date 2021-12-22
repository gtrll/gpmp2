import time
import numpy as np
from gtsam import *
from gpmp2 import *

import argparse


from jist.datasets.generate2Ddataset import *
from jist.jist import *
import hydra
import sys

import pickle


import hydra
from random import Random


# python pointRobot_test.py dataset.vel_limit=1.5 dataset.obstacle_num=80 problem.node_budget=60 problem.sdf_side=60.0 

def sample_start_goal_random_env(hydra_cfg, seed_val):
    
    width = 120.0
    height = 90.0
    min_dist = 70.0

    rand_generator = Random()
    rand_generator.seed(seed_val)    
    while True:

        if not hydra_cfg.problem.use_theta:
            start_conf = np.asarray(
                [width * rand_generator.random(), height * rand_generator.random()]
            )
            goal_conf = np.asarray(
                [width * rand_generator.random(), height * rand_generator.random()]
            )
        else:
            start_conf = np.asarray(
                [
                    width * rand_generator.random(),
                    height * rand_generator.random(),
                    np.pi * rand_generator.random(),
                ]
            )
            goal_conf = np.asarray(
                [
                    width * rand_generator.random(),
                    height * rand_generator.random(),
                    np.pi * rand_generator.random(),
                ]
            )

        if np.linalg.norm(start_conf[:2]- goal_conf[:2]) > min_dist:
            return start_conf, goal_conf


@hydra.main(config_path="configs/default_config.yaml")
def main(hydra_cfg):

    total_experiments = 30
    plot_mode = "suppress"

    print(hydra_cfg.pretty())

    algorithms = {"rrt": rrt_chain}

    for algo in algorithms:
        print(
            "###########################################################################"
        )
        print("Running the algo:", algo)
        print(
            "###########################################################################"
        )
        results = []
        exp_id = 0
        seed_val = hydra_cfg.problem.seed_val

        num_succs = 0
        
        while exp_id < total_experiments:

            
            print("Experiment Number: ", exp_id)

            start_conf, goal_conf = sample_start_goal_random_env(hydra_cfg, seed_val)

            res = Result()
            res.seed_val = seed_val
            res.start_conf = start_conf
            res.goal_conf = goal_conf
            res.obstacle_num = hydra_cfg.dataset.obstacle_num

            dataset = Dynamic2Ddataset()  # dataset_func()
            dataset.init_obstacles(
                seed_val=seed_val + 1, obstacle_num=hydra_cfg.dataset.obstacle_num,
                vel_limit=hydra_cfg.dataset.vel_limit,
                acc_limit=hydra_cfg.dataset.acc_limit,
                start = start_conf,
                goal = goal_conf
            )
            problem = make_problem_from_config(hydra_cfg.problem, start_conf, goal_conf)

            (
                res.status,
                res.iterations,
                res.optimizer_time,
                res.total_time,
                res.distance,
                res.vels,
                res.goal_dist,
            ) = algorithms[algo](
                start_conf, goal_conf, dataset, problem, plot_mode=plot_mode
            )

            print(
                res.status,
                res.iterations,
                res.optimizer_time,
                res.total_time,
                start_conf,
                goal_conf,
                res.goal_dist,
            )

            seed_val += 1
            if res.status != test_status.INVALIDSTART:
                results.append(res)
                exp_id += 1


            if res.status == test_status.SUCCESS:
                num_succs += 1

        file_name = algo + ".p"
        pickle.dump(results, open(file_name, "wb"))

        print("#####################################################")
        print("Success Rate:", float(num_succs)/ float(total_experiments))
        print("#####################################################")


if __name__ == "__main__":

    main()
