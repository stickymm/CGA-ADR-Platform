from Colins_Nav import GateMission, NavigationController
class testMission(GateMission):
    def run(self, nav: NavigationController):
        gateNum : int = 1
        gate_count = 0

        print("starting turn test")

        while nav.running and gate_count < gateNum:
            print("beginning the search")
            gate = self.observe_gate(nav, duration = 3.0)
            nav.turn_test(gate, 3.0)
        
        if gate_count >= gateNum:
            nav.land()
