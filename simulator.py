# Authors : Gastone Pietro Rosati Papini
# Date    : 09/08/2022
# License : MIT
import argparse
import math
import signal

from pydrivingsim import World
from scenarios import BasicSpeedLimit, BasicTrafficLight, OnlyVehicle, AutonomousVehicle, GetTheCoins, MapScenario

class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)

  def exit_gracefully(self, *args):
    self.kill_now = True

def main():
    parser = argparse.ArgumentParser(description="PyDrivingSim")
    parser.add_argument("--map", help="off-road map file (JSON), e.g. scenarios/maps/offroad_boulder_field.json")
    parser.add_argument("--preview", action="store_true", help="with --map: show the map without connecting to the agent")
    args = parser.parse_args()

    if args.map:
        av = MapScenario(args.map, use_agent=not args.preview)
    else:
        # Enable this to test only single vehicle
        #av = OnlyVehicle()
        av = AutonomousVehicle()
        BasicTrafficLight()
        # Enable this to test the coins
        #GetTheCoins()
        # Enable this to test the speed limit
        BasicSpeedLimit()

    killer = GracefulKiller()
    while not killer.kill_now and World().loop:
        av.update()
        World().update()

    av.terminate()
    World().exit()

main()