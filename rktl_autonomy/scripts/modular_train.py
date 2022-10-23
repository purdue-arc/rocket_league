from train_rocket_league import train
import yaml
from rktl_autonomy import EnvCounter
import sys
import os
from pathlib import Path
from threading import Thread

if __name__ == '__main__':
    numEnvsAllowed = 24
    if len(sys.argv) == 2:
        numEnvsAllowed = int(sys.argv[1])
    configFolder = os.path.join(os.path.join(os.path.join(Path(__file__), os.pardir), os.pardir), Path("config"))
    configFile = Path(os.path.abspath(os.path.join(configFolder, "rocket_league.yaml")))
    print(os.path.abspath(configFile))
    file = yaml.load(open(configFile), Loader=yaml.FullLoader)
    numGroups = len(file["reward"]["win"])
    for i in range(numGroups):
        args = (int(numEnvsAllowed / numGroups), 100, 240000000, EnvCounter())
        thread = Thread(target=train, args=args)
        thread.start()
