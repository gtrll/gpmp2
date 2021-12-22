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

# python pointRobot_test_patrol_guard.py dataset.vel_limit=1.4 dataset.obstacle_num=2 problem.node_budget=100 problem.sdf_side=60.0 problem=example_patrol dataset=patrol_guard

def sample_start_goal_patrol_guard(hydra_cfg, seed_val):

    rand_generator = Random()
    rand_generator.seed(seed_val)    


    x_bounds = [7.5, 12.5]
    y_bounds = [2.0, 5.0]
    start_conf = np.asarray(
        [
            x_bounds[0] + (x_bounds[1] - x_bounds[0]) * rand_generator.random(),
            y_bounds[0] + (y_bounds[1] - y_bounds[0]) * rand_generator.random(),
            np.pi * rand_generator.random(),
        ]
    )

    y_bounds = [32.5, 37.5]

    goal_conf = np.asarray(
        [
            x_bounds[0] + (x_bounds[1] - x_bounds[0]) * rand_generator.random(),
            y_bounds[0] + (y_bounds[1] - y_bounds[0]) * rand_generator.random(),
            np.pi * rand_generator.random(),
        ]
    )

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

            start_conf, goal_conf = sample_start_goal_patrol_guard(hydra_cfg, seed_val)

            res = Result()
            res.seed_val = seed_val
            res.start_conf = start_conf
            res.goal_conf = goal_conf
            res.obstacle_num = hydra_cfg.dataset.obstacle_num

            dataset = Patrol2Ddataset()  # dataset_func()
            dataset.init_obstacles(
                seed_val=seed_val + 1, obstacle_num=hydra_cfg.dataset.obstacle_num,
                vel_limit=hydra_cfg.dataset.vel_limit,
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
