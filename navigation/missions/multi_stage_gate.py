import os
import sys

from ..navigation import GateMission, NavigationController

class MultiStageGateMission(GateMission):
    """Default race mission that approaches each gate in shrinking stages."""

    def run(self, nav: NavigationController):
        """Fly the 3m -> 2m -> 1m -> pass-through sequence for up to 8 gates."""
        gate_count = 0

        while nav.running and gate_count < 8:
            print("\n==============================")
            print(f"[*] Looking for Gate {gate_count + 1} of 8")
            print("==============================")

            gate = self.observe_gate(nav, duration=3.0)
            if not gate:
                nav.turn_around_180()
                continue

            target_3m = self.build_standoff_target(nav, gate, standoff_m=3.0)
            nav.move_to_target(target_3m, "3m Standoff", max_speed_m_s= 0.15)

            gate = self.observe_gate(nav, duration=6.0)
            if not gate:
                print("[!] Lost gate at 3m. Restarting.")
                continue

            target_2m = self.build_standoff_target(nav, gate, standoff_m=2.0)
            nav.move_to_target(target_2m, "2m Standoff", max_speed_m_s= 0.15)

            gate = self.observe_gate(nav, duration=6.0)
            if not gate:
                print("[!] Lost gate at 2m. Restarting.")
                continue

            target_1m = self.build_standoff_target(nav, gate, standoff_m=1.0)
            nav.move_to_target(target_1m, "1m Standoff", max_speed_m_s= 0.15)

            gate = self.observe_gate(nav, duration=6.0)
            if not gate:
                print("[!] Lost gate right before pass. Restarting.")
                continue

            pass_target = self.build_pass_through_target(nav, gate, pass_dist_m=1.5)
            nav.move_to_target(pass_target, "Through The Gate!", max_speed_m_s= 0.15)

            gate_count += 1
            print(f"[*] Successfully navigated Gate {gate_count}!")

        if gate_count >= 8:
            nav.land()
