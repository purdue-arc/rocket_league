#!/usr/bin/env python3

from train_rocket_league import train
import yaml
import sys
import os
from multiprocessing import Process

if __name__ == '__main__':
    numEnvsAllowed = 24

    if len(sys.argv) == 2:
        numEnvsAllowed = int(sys.argv[1])

    configFile = os.path.join(os.pardir, 'config', 'rocket_league.yaml')
    print(os.path.abspath(configFile))

    file = yaml.load(open(configFile), Loader=yaml.FullLoader)
    numGroups = len(file["reward"]["win"])

    for i in range(numGroups):
        args = (int(numEnvsAllowed / numGroups), 100, 240000000, i)
        p = Process(target=train, args=args)
        p.start()
        print(f'Starting thread {i}/{numGroups}')
