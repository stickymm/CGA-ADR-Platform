from Colins_Nav import GateMission, NavigationController, GateDetection
class testMission(GateMission):
    def run(self, nav: NavigationController):
        gateNum : int = 1
        gate_count = 0

        print("starting turn test")
        gate1 = GateDetection(0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0)

        target1 = self.build_standoff_target(nav, gate1, 1.0)
        nav.move_to_target(target1, "running")
        
        while nav.running and gate_count < gateNum:
            print("beginning the search")
            gate = self.observe_gate(nav, duration = 3.0)
            nav.turn_test(gate, 3.0)
        
        if gate_count >= gateNum:
            nav.land()
