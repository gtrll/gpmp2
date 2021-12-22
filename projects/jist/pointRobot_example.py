import numpy as np
import argparse


from jist.datasets.generate2Ddataset import *
from jist.jist import rrt_chain, make_problem_from_config

import hydra
import sys

#python pointRobot_example.py problem.node_budget=100 problem.seed_val=1 dataset.obstacle_num=40 problem.sdf_side=65.0 dataset.vel_limit=1.5

@hydra.main(config_path="configs/default_config.yaml")
def main(hydra_cfg):

    print(hydra_cfg.pretty())

    plot_mode = "debug"
    start_conf = np.asarray([10.0, 20.0, 0.0])  # np.asarray([10, 20, 0])
    goal_conf = np.asarray([40.0, 60.0, np.pi / 2])  # np.asarray([20, 40, 0])

    seed_val = hydra_cfg.problem.seed_val


    print("########## Running JIST #########################")
    # dataset_func = getattr(sys.modules[__name__], hydra_cfg.dataset.name)
    dataset = Dynamic2Ddataset()  # dataset_func()
    dataset.init_obstacles(
        seed_val=seed_val + 1, obstacle_num=hydra_cfg.dataset.obstacle_num
    )
    problem = make_problem_from_config(hydra_cfg.problem, start_conf, goal_conf)
    print(rrt_chain(start_conf, goal_conf, dataset, problem, plot_mode=plot_mode))



if __name__ == "__main__":
    main()
