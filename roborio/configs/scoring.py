class ScoringConfigs:
    def __init__(
        self,
        elevator_pos: float = 0.0,
        shoulder_start_pos: float = -0.25,
        shoulder_end_pos: float = -0.25,
        arm_pos: float = 0.0,
        grabber_v: float = 0.0,
        configName: str = "Nuthin :)",
    ):
        self.elevator_pos = elevator_pos
        self.shoulder_start_pos = shoulder_start_pos
        self.shoulder_end_pos = shoulder_end_pos
        self.arm_pos = arm_pos
        self.grabber_v = grabber_v
        self.configName = configName


L1_shoot = ScoringConfigs(4.0, -0.1, -0.1, 3.0, -12, "L1 Shoot")
L2_shoot = ScoringConfigs(0, 0.12, 0.12, 0, 12, "L2 Shoot")
L3_shootV1 = ScoringConfigs(3, 0.2, 0.2, 2, 12, "L3V1 Shoot")
L3_shootV2 = ScoringConfigs(5, 0.12, 0.12, 0, 12, "L3V2 Shoot")
L3_dunk = ScoringConfigs(8, 0.08, -0.06, 0, 0, "L3 Dunk")
L4_dunk = ScoringConfigs(13, 0.2, -0.03, 2, 0, "L4 Dunk")
L4_shoot = ScoringConfigs(13, 0.22, 0.22, 3, 12, "L4 Shoot")
algae_L23 = ScoringConfigs(6.0, -0.03, -0.03, 0.0, 8.0, "Algae L23")
algae_L34 = ScoringConfigs(13.5, -0.03, -0.03, 0.0, 8.0, "Algae L34")
algae_process = ScoringConfigs(0.0, -0.055, -0.055, 0.0, -12.0, "Algae Process")
