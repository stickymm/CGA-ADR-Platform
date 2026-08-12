import os
import sys

from Colins_Nav import GateMission, NavigationController, GateDetection
import math

class advSquare(GateMission):
    def run(self, nav: NavigationController):
        targets = []
        gates = []

        while len(gates) < 4:
            gates = self.find_gates()
        
        for i in 4:
            targets[i] = self.build_standoff_target(nav, gates[i], 0)
        
        run : bool = nav.adv_run_square(targets[0], targets[1], targets[2], targets[3], 4.0)
            
        if run:
            nav.land()
