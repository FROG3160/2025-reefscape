from commands2 import Command
from subsystems.grabber import Grabber


class ScoringCommand(Command):
    def __init__(self, grabber: Grabber, table: str = "Undefined") -> None:
        super().__init__()
        self.grabber = grabber
        self.addRequirements(self.grabber)

    def execute(self):
        scoring_config = self.grabber.scoring_config
        self.grabber._run(scoring_config.get_grabber_voltage())

    def isFinished(self):
        scoring_config = self.grabber.scoring_config
        if scoring_config.get_element() == "Algae":
            if scoring_config.get_grabber_voltage() < 0:
                # We are using negative voltage to push out algae
                # return true when it's gone.
                return self.grabber._not_detecting_algae()
            else:
                return self.grabber._detecting_algae()
        else:
            return self.grabber._not_detecting_coral_range()

    def end(self, interrupted):
        scoring_config = self.grabber.scoring_config
        if scoring_config.get_element() == "Algae":
            if self.grabber._detecting_algae():
                # if we are still detecting algae, we just grabbed it.  Reduce
                # motor voltage by 1/4
                self.grabber._run(scoring_config.get_grabber_voltage() * 0.75)
            else:
                # self.grabber._not_detecting_algae()
                self.grabber.stop_motor()
        else:
            self.grabber.stop_motor()
