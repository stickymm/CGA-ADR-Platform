from Colins_Nav import GateMission, NavigationController, GateDetection
import math
class mediumSquare(GateMission):
    def run(self, nav: NavigationController):
        targets = []
        gates = []
        adjust = math.pi/12


        while len(gates) < 4:
            gates = self.find_gates()
        
        for i in 4:
            gate : GateDetection = gates[i]
            gate.yaw_deg -= math.degrees(adjust)
            targets[i] = self.build_standoff_target(nav, gates[i], 0.5)

        gateNum : int = 4
        gate_count = 0
        
        move_dist : int = 8 # meters

        while nav.running and gate_count < gateNum:
            nav.adv_move_to_target(targets[gate_count], f"target {gate_count+1} prep", vfn = 0.0, vfe = 0.0, vfd = 0.0, theta_f= math.radians(gates[gate_count].yaw_deg)-adjust)
            
            falseTarg = gates[gate_count]

            falseTarg.yaw_deg -= adjust

            post = self.build_pass_through_target(nav, falseTarg, move_dist)

            nav.adv_move_to_target(post, f"ready for target {gate_count+2}", vfn = 1.0, vfe = 0, vfd=0, theta_f=nav.get_vehicle_snapshot().yaw_rad)
            
            falseTarg2 = gates[gate_count+1]
            
            falseTarg2.yaw_deg -= adjust

            post2 = self.build
            nav.move_to_target_curve()
            
            gate_count += 1

        if gate_count >= gateNum:
            nav.land()

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
