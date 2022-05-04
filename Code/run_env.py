"""
Author: David Valencia
Date:   03/ 05 /2022

Describer:
            This file runs the robot test bed RL environment very straight forward, following the same concept of
            a Gym-OpenAi env environment
            The core functions are in the main_rl_env.py file
"""

import time
from main_rl_env import *

num_episodes     = 3
episode_horizont = 5


def main(args=None):

    for episode in range(num_episodes):
        reset_env()
        print("Sending Robot to Home Position")
        time.sleep(2.0)

        for step in range(episode_horizont):
            print(f"-------Episode:{episode+1} Step:{step+1}---------")
            state_space = state_space_funct()
            action      = generate_random_act()
            env_step()  # take the action
            #reward, done = calculate_reward()
            next_state = state_space_funct()
            time.sleep(2.0)

        print(f"******* -----Episode {episode+1} Ended----- ********* ")

    motor_terminate()


if __name__ == '__main__':
    main()