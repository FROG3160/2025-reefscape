class ScoringConfigs:
    def __init__(
        self,
        elevator_pos: float = 0.0,
        shoulder_start_pos: float = -0.25,
        shoulder_end_pos: float = -0.25,
        arm_pos: float = 0.0,
        grabber_v: float = 0.0,
    ):
        self.elevator_pos = elevator_pos
        self.shoulder_start_pos = shoulder_start_pos
        self.shoulder_end_pos = shoulder_end_pos
        self.arm_pos = arm_pos
        self.grabber_v = grabber_v


CORAL_EJECT_UP = (
    -12.0
)  # Voltage to move coral UP out of the grabber when arm is out to the side
CORAL_EJECT_DOWN = 12.0
ALGAE_EJECT = -12.0
ALGAE_INTAKE = 8.0

ELEVATOR_L1 = 4.0
ELEVATOR_L2 = 0.0
ELEVATOR_L3 = 5.0
ELEVATOR_L3_DUNK = 8.0
ELEVATOR_L4 = 13.0
ALGAE_L2 = 6.0
ALGAE_L3 = 13.5
ALGAE_PROCESS = 0.0

SHOULDER_L1 = -0.1
SHOULDER_L2_3 = 0.12
SHOULDER_L3_DUNK = 0.08
SHOULDER_L3_DUNK_2 = -0.06
SHOULDER_L4_DUNK = 0.2
SHOULDER_L4_DUNK_2 = -0.03
SHOULDER_L4_SHOOT = 0.22
SHOULDER_L2_3_ALGAE = -0.03
SHOULDER_PROCESS = -0.055

ARM_L1 = 3.0
ARM_L4_SHOOT = 3.0


L1_shoot = ScoringConfigs(ELEVATOR_L1, SHOULDER_L1, SHOULDER_L1, ARM_L1, CORAL_EJECT_UP)
L2_shoot = ScoringConfigs(
    ELEVATOR_L2, SHOULDER_L2_3, SHOULDER_L2_3, 0, CORAL_EJECT_DOWN
)
# L3_shootV1 = ScoringConfigs(3, 0.2, 0.2, 2, CORAL_EJECT_DOWN)
L3_shootV2 = ScoringConfigs(
    ELEVATOR_L3, SHOULDER_L2_3, SHOULDER_L2_3, 0, CORAL_EJECT_DOWN
)
L3_dunk = ScoringConfigs(ELEVATOR_L3_DUNK, SHOULDER_L3_DUNK, SHOULDER_L3_DUNK_2, 0, 0)
L4_dunk = ScoringConfigs(ELEVATOR_L4, SHOULDER_L4_DUNK, SHOULDER_L4_DUNK_2, 0)
L4_shoot = ScoringConfigs(
    ELEVATOR_L4, SHOULDER_L4_SHOOT, SHOULDER_L4_SHOOT, ARM_L4_SHOOT, CORAL_EJECT_DOWN
)
algae_L23 = ScoringConfigs(
    ALGAE_L2, SHOULDER_L2_3_ALGAE, SHOULDER_L2_3_ALGAE, 0.0, ALGAE_INTAKE
)
algae_L34 = ScoringConfigs(
    ALGAE_L3, SHOULDER_L2_3_ALGAE, SHOULDER_L2_3_ALGAE, 0.0, ALGAE_INTAKE
)

algae_process = ScoringConfigs(
    ALGAE_PROCESS, SHOULDER_PROCESS, SHOULDER_PROCESS, 0.0, ALGAE_EJECT
)
