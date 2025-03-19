class ScoringConfigs:
    def __init__(
        self,
        elevator_pos: float,
        shoulder_start_pos: float,
        shoulder_end_pos: float,
        arm_pos: float,
        grabber_v: float,
    ):
        self.elevator_pos = elevator_pos
        self.shoulder_start_pos = shoulder_start_pos
        self.shoulder_end_pos = shoulder_end_pos
        self.arm_pos = arm_pos
        self.grabber_v = grabber_v


L1_shoot = ScoringConfigs(4.0, -0.1, -0.1, 3.0, -12)
L2_shoot = ScoringConfigs(0, 0.12, 0.12, 0, 12)
L3_shootV1 = ScoringConfigs(3, 0.2, 0.2, 2, 12)
L3_shootV2 = ScoringConfigs(5, 0.12, 0.12, 0, 12)
L3_dunk = ScoringConfigs(8, 0.08, -0.06, 0, 0)
L4_dunk = ScoringConfigs(13, 0.2, -0.03, 2, 0)
L4_shoot = ScoringConfigs(13, 0.22, 0.22, 3, 12)
