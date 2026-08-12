import os
import sys

from ..navigation import GateMission, NavigationController, GateDetection

class MultiStageGateMission(GateMission):
    """Default race mission that approaches each gate in shrinking stages."""
    
    
    def run(self, nav: NavigationController):
        """Fly the 3m -> 2m -> 1m -> pass-through sequence for up to 8 gates."""
        gateNum : int = 4
        gate_count : int= 0
        maxDist_m : int = 4
        while nav.running and gate_count < gateNum:
            print("\n==============================")
            print(f"[*] Looking for Gate {gate_count + 1} of {gateNum}")
            print("==============================")
            dir : str = "right" 
            #change if necessary
            if(gate_count % 2 == 0):
                dir = "left"
            
            gate : GateDetection = self.observe_gate(nav, duration=3.0)
            while not gate or gate.dist > maxDist_m:
                nav.turn_around_45(dir)
                gate = self.observe_gate(nav, duration=3.0)
                continue

            target_3m = self.build_standoff_target(nav, gate, standoff_m=3.0)
            nav.move_to_target(target_3m, "3m Standoff", max_speed_m_s= 0.15)

            gate : GateDetection = self.observe_gate(nav, duration=6.0)
            
            while not gate or gate.dist > maxDist_m:
                nav.turn_around_45(dir)
                print("[!] Lost gate at 3m. Restarting.")
                gate = self.observe_gate(nav, duration=6.0)
                continue

            target_2m = self.build_standoff_target(nav, gate, standoff_m=2.0)
            nav.move_to_target(target_2m, "2m Standoff", max_speed_m_s= 0.15)

            gate : GateDetection = self.observe_gate(nav, duration=6.0)
            while not gate or gate.dist > maxDist_m:
                nav.turn_around_45(dir)
                print("[!] Lost gate at 2m. Restarting.")
                gate = self.observe_gate(nav, duration=6.0)
                continue

            target_1m = self.build_standoff_target(nav, gate, standoff_m=1.0)
            nav.move_to_target(target_1m, "1m Standoff", max_speed_m_s= 0.15)

            gate : GateDetection = self.observe_gate(nav, duration=6.0)
            
            while not gate or gate.dist > maxDist_m:
                nav.turn_around_45(dir)
                print("[!] Lost gate right before pass. Restarting.")
                gate = self.observe_gate(nav, duration=6.0)
                continue

            pass_target = self.build_pass_through_target(nav, gate, pass_dist_m=1.5)
            nav.move_to_target(pass_target, "Through The Gate!", max_speed_m_s= 0.15)

            gate_count += 1
            print(f"[*] Successfully navigated Gate {gate_count}!")

        if gate_count >= gateNum:
            """
            have drone move out of the target before landing
            maybe check a height graph to ensure gate is passed and drone can land
            or maybe have drone fly small circular path so the downward sensor
            can see its full landing area but that is probably not necessary since
            the camera should be able to determine the size of the bottom area and 
            whether it is wide enough or to do image processing to determine if there
            is something in the middle of the screen
            """            
            nav.land()
