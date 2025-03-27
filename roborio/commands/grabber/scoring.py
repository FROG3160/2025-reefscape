from commands2 import Command
from subsystems.grabber import Grabber


class ScoringCommand(Command):
    def __init__(self, grabber: Grabber, table: str = "Undefined") -> None:
        super().__init__()
        self.grabber = grabber
        self.addRequirements(self.grabber)

    def execute(self):
        scoring_config = self.grabber.scoring_config
        if scoring_config.get_element == "Algae" and self.grabber._detecting_algae():
            self.grabber._run(scoring_config.get_grabber_voltage() * 0.75)
        else:
            self.grabber._run(scoring_config.get_grabber_voltage())

    def isFinished(self):
        scoring_config = self.grabber.scoring_config
        if (
            scoring_config.get_element() == "Algae"
            and scoring_config.get_grabber_voltage() < 0
        ):
            return self.grabber._not_detecting_algae()
        if scoring_config.get_element() == "Coral":
            return self.grabber._not_detecting_coral()
        return False

    def end(self, interrupted):
        self.grabber.stop_motor()
