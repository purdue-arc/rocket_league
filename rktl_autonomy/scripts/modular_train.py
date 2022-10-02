from train_rocket_league import train
import yaml
from rktl_autonomy import EnvCounter
import sys
from pathlib import Path

if __name__ == '__main__':
    numEnvsAllowed = 24
    if len(sys.argv) == 2:
        numEnvsAllowed = int(sys.argv[1])
    file = yaml.load(open(Path(__file__).parent.parent / "config" / "rocket_league.yaml"), Loader=yaml.FullLoader)
    numGroups = len(file["reward"]["win"])
    for i in range(numGroups):
        train(n_envs=numEnvsAllowed / numGroups, env_counter=EnvCounter())
