

class Arm:
    def __init__(self):
        self.motor = FROGTalonFX(
            id=constants.kArmMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(90),
                slot0gains=Slot0Configs(),
                
            ).with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE)
            ),
            self.limitswitch = limitswitch()
            parent_nt="Arm",
            motor_name="motor",
        )
       