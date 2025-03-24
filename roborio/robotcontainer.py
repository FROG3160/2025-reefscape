#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import os
import wpilib
from wpilib import DriverStation, SmartDashboard
from wpilib.interfaces import GenericHID
from wpimath.units import degreesToRadians
from wpimath.geometry import Pose2d, Rotation2d

import commands2
import commands2.button
import commands2.cmd
from constants import (
    kDriverControllerPort,
    kOperatorControllerPort,
    kDeadband,
    kDebouncePeriod,
    kTranslationSlew,
    kRotSlew,
    kCamera1Name,
    kCamera2Name,
    robotToCamera1,
    robotToCamera2,
)
from commands2.cmd import runOnce, startEnd, waitUntil
from commands2 import (
    DeferredCommand,
    PrintCommand,
    RepeatCommand,
    InterruptionBehavior,
    Command,
    WaitCommand,
)
from commands2.sysid import SysIdRoutine

from FROGlib.xbox import FROGXboxDriver, FROGXboxTactical
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from configs.scoring import (
    ScoringConfigs,
    L1_shoot,
    L2_shoot,
    # L3_shootV1,
    L3_shootV2,
    L3_dunk,
    L4_shoot,
    L4_dunk,
    algae_L23,
    algae_L34,
    algae_process,
)
from subsystems.drivechassis import DriveChassis
from subsystems.positioning import Position
from subsystems.vision import VisionPose
from subsystems.lift import Lift
from subsystems.shoulder import Shoulder
from subsystems.grabber import Grabber
from subsystems.arm import Arm
from subsystems.climber import Climber
from pathplannerlib.auto import AutoBuilder, NamedCommands

from commands.drive.field_oriented import (
    ManualDrive,
    AutoMoveOffLine,
)
from commands.drive.robot_oriented import (
    ManualRobotOrientedDrive,
)


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    first_shoulder_str = "First Shoulder Pos"
    second_shoulder_str = "Second Shoulder Pos"
    elevator_str = "Elevator Pos"
    arm_str = "Arm Pos"
    grabber_str = "Grabber Voltage"

    def __init__(self) -> None:

        # The driver's controller
        self.driverController = FROGXboxDriver(
            kDriverControllerPort,
            kDeadband,
            kDebouncePeriod,
            kTranslationSlew,
            kRotSlew,
        )
        self.tacticalController = FROGXboxTactical(
            kOperatorControllerPort,
            kDeadband,
        )
        # self.operatorController = FROGXboxOperator(kOperatorControllerPort, kDeadband)

        # Create all subsystems here.  If a subsystem is needed by other subsystems, create it first,
        # then pass it in to the subsystems needing it.

        # List of all positioning cameras
        self.positioningCameras = []

        # Create all positioning cameras here
        self.camera1 = VisionPose(kCamera1Name, robotToCamera1)
        self.camera2 = VisionPose(kCamera2Name, robotToCamera2)

        # Sensors/Cameras
        # Add each positioning camera to the positioningCameras list
        self.positioningCameras.append(self.camera1)
        self.positioningCameras.append(self.camera2)
        self.scoring_config = ScoringConfigs()
        self.positioning = Position()

        # Subsystems
        self.driveSubsystem = DriveChassis(self.positioningCameras)
        self.elevator = Lift()
        self.shoulder = Shoulder()
        self.arm = Arm()
        self.grabber = Grabber()
        # self.intake = Intake()
        self.climber = Climber()

        self.registerNamedCommands()

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.driveSubsystem.setDefaultCommand(
            ManualDrive(self.driverController, self.driveSubsystem)
        )
        self.subsystems_homed = False

        # AUTO CHOOSER
        self.autochooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("PathPlanner Autos", self.autochooser)
        SmartDashboard.putData("Elevator", self.elevator)
        SmartDashboard.putData("Shoulder", self.shoulder)
        SmartDashboard.putData("Arm", self.arm)
        SmartDashboard.putData("Grabber", self.grabber)

    def registerNamedCommands(self):
        # NamedCommands.registerCommand(
        #     "Place L1 coral", self.full_auto_scoring_sequence(L1_shoot)
        # )
        NamedCommands.registerCommand(
            "Set L1 Scoring", self.set_scoring_config(L1_shoot)
        )
        NamedCommands.registerCommand("Move to Position", self.move_to_position())
        NamedCommands.registerCommand("Score", self.score())
        NamedCommands.registerCommand("Stop Grabber", self.grabber.stop_motor())
        NamedCommands.registerCommand("Home Systems", self.move_to_home())

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        self.configureDriverControls()
        self.configureOperatorControls()
        self.configureAutomationBindings()

    def configureTestBindings(self):
        """Configures trigger bindings for use in test mode"""
        SmartDashboard.putNumber(self.first_shoulder_str, 0.2)
        SmartDashboard.putNumber(self.second_shoulder_str, 0.08)
        SmartDashboard.putNumber(self.elevator_str, 8)
        SmartDashboard.putNumber(self.arm_str, 0)
        SmartDashboard.putNumber(self.grabber_str, 0)

        SmartDashboard.putData(
            "Position For Coral Placement",
            DeferredCommand(lambda: self.test_move_all()),
        )
        SmartDashboard.putData(
            "Run Grabber", DeferredCommand(lambda: self.test_run_grabber())
        )
        SmartDashboard.putData("Stop Grabber", self.grabber.run_motor(0))
        SmartDashboard.putData(
            "Move Elevator To Pos", DeferredCommand(lambda: self.test_move_elevator())
        )
        SmartDashboard.putData(
            "Move Shoulder to second Pos",
            DeferredCommand(lambda: self.test_move_shoulder()),
        )
        SmartDashboard.putData(
            "Move Arm to Pos", DeferredCommand(lambda: self.test_move_arm())
        )

        self.driverController.leftBumper().whileTrue(self.grabber.run_motor(0))

    def _set_scoring_config(self, scoringConfig: ScoringConfigs):
        for sub in [self.shoulder, self.arm, self.elevator, self.grabber, self]:
            sub.scoring_config = scoringConfig
        SmartDashboard.putString("Scoring Config value", self.scoring_config.configName)

    def set_scoring_config(self, scoringConfig) -> Command:
        return runOnce(lambda: self._set_scoring_config(scoringConfig))

    def position_for_coral_placement(
        self, first_shoulder_pos, elevator_pos, second_shoulder_pos, arm_pos
    ) -> Command:
        return (
            self.shoulder.move(first_shoulder_pos)
            .andThen(waitUntil(lambda: self.shoulder.at_position(first_shoulder_pos)))
            .andThen(self.elevator.move(elevator_pos))
            .andThen(waitUntil(lambda: self.elevator.at_position(elevator_pos)))
            .andThen(self.shoulder.move(second_shoulder_pos))
            .andThen(self.arm.move(arm_pos))
        )

    def test_move_all(self) -> Command:

        first_shoulder_pos = SmartDashboard.getNumber(self.first_shoulder_str, -0.25)
        elevator_pos = SmartDashboard.getNumber(self.elevator_str, 0)
        second_shoulder_pos = SmartDashboard.getNumber(self.second_shoulder_str, -0.25)
        arm_pos = SmartDashboard.getNumber(self.arm_str, 0)

        return (
            self.shoulder.move(first_shoulder_pos)
            .andThen(waitUntil(lambda: self.shoulder.at_position(first_shoulder_pos)))
            .andThen(self.elevator.move(elevator_pos))
            .andThen(waitUntil(lambda: self.elevator.at_position(elevator_pos)))
            .andThen(self.shoulder.move(second_shoulder_pos))
            .andThen(self.arm.move(arm_pos))
        )

    def grab_coral_from_trough(self) -> Command:
        return (
            self.arm.move(0)
            .andThen(waitUntil(self.arm.at_home))
            .andThen(self.shoulder.move(-0.25))
            .andThen(waitUntil(lambda: self.shoulder.at_position(-0.25)))
            .andThen(self.arm.move(0.8))
            .andThen(self.grabber.intake_coral())
            .andThen(self.arm.move(0))
        )

    def hold_coral_during_travel(self) -> Command:
        return (
            self.grabber.run_motor(0)
            .andThen(self.arm.move(0))
            .alongWith(self.shoulder.move(-0.2))
        )

    def move_to_home(self) -> Command:
        return (
            self.grabber.stop_motor()
            .andThen(self.arm.move(0))
            .andThen(waitUntil(self.arm.at_home))
            .andThen(self.elevator.move(0))
            .alongWith(self.shoulder.move(-0.2))
        )

    def move_to_station(self) -> Command:
        return (
            self.arm.move(0)
            .andThen(self.shoulder.move(-0.2))
            .alongWith(self.elevator.move(10))
        )

    def hold_algae_during_travel(self) -> Command:
        return self.arm.move(0).alongWith(self.shoulder.move(-0.15))

    def move_to_position(self) -> Command:
        return (
            self.shoulder.move_to_scoring_start()
            # .andThen(
            #     waitUntil(
            #         lambda: self.shoulder.at_position(
            #             self.scoring_config.get_shoulder_start_position()
            #         )
            #     )
            # )
            .andThen(self.elevator.move_to_scoring())
            # .andThen(
            #     waitUntil(
            #         lambda: self.elevator.at_position(
            #             self.scoring_config.get_elevator_position()
            #         )
            #     )
            # )
            .andThen(self.arm.move_to_scoring())
        )

    def score(self) -> Command:
        # second_shoulder_pos = self.scoringConfig.shoulder_end_pos
        # grabber_voltage = self.scoringConfig.grabber_v
        return self.grabber.run_scoring()

        # if self.scoring_config == L3_dunk or self.scoring_config == L4_dunk:
        #     return self.shoulder.move_with_variable(
        #         self.scoring_config.get_shoulder_end_position
        #     )
        # elif self.scoring_config == L1_shoot:
        #     return self.grabber.eject_coral_L1()
        # else:
        #     return self.grabber.run_motor_with_variable(
        #         self.scoring_config.get_grabber_voltage
        #     )

    # def full_auto_scoring_sequence(self, scoringConfig: ScoringConfigs) -> Command:
    #     return (
    #         self.setScoringAction(scoringConfig)
    #         .andThen(DeferredCommand(lambda: self.move_to_position()))
    #         .andThen(DeferredCommand(lambda: self.move_to_score()))
    #     )

    def test_run_grabber(self) -> Command:
        grabber_voltage = SmartDashboard.getNumber(self.grabber_str, 0)
        return self.grabber.run_motor(grabber_voltage)

    def test_move_elevator(self) -> Command:
        elevator_pos = SmartDashboard.getNumber(self.elevator_str, 0)
        return self.elevator.move(elevator_pos)

    def test_move_shoulder(self) -> Command:
        shoulder_pos = SmartDashboard.getNumber(self.second_shoulder_str, 0)
        return self.shoulder.move(shoulder_pos)

    def test_move_arm(self) -> Command:
        arm_pos = SmartDashboard.getNumber(self.arm_str, 0)
        return self.arm.move(arm_pos)

    def home_subsystems(self):
        if not self.subsystems_homed:
            self.elevator.home().schedule()
            self.arm.set_home().schedule()
            # self.intake.set_home().schedule()
            self.systems_homed = True

    def move_off_line(self) -> Command:
        return WaitCommand(1.5).deadlineWith(AutoMoveOffLine(self.driveSubsystem))

    def configureAutomationBindings(self):
        """Configures all triggers that are watching states or conditions
        of various subsystems.
        """

        # self.intake.intake_deployed().onTrue(
        #     self.shoulder.move(self.shoulder.Position.READY).andThen(
        #         self.intake.start_intake()
        #     )
        # )

        # self.intake.coral_detected_trigger().onFalse(
        #     self.intake.set_intake_loaded().andThen(
        #         PrintCommand("CORAL DETECTED WENT FALSE")
        #     )
        # )

        """CORAL LOADED TRIGGER CAN BE REPLACED WITH HAS_STATE LIKE LINE 172"""
        # self.intake.has_state(self.intake.State.CORAL_LOADED).onTrue(
        #     self.intake.stop_intake().andThen(
        #         self.shoulder.move(self.shoulder.Position.LOAD)
        #     )
        # )
        # self.shoulder.at_position(self.shoulder.Position.LOAD).and_(
        #     self.intake.has_state(self.intake.State.CORAL_LOADED)
        # ).onTrue(
        #     self.grabber.intake_coral().andThen(
        #         self.arm.move(self.arm.Position.CORAL_PICKUP)
        #     )
        # )

    def configureDriverControls(self):
        """Configures triggers for manual control by the driver"""
        # self.driverController.start().onTrue(
        #     runOnce(lambda: self.driveSubsystem.setFieldPositionFromVision())
        # )
        # self.configureSysIDButtonBindings()

        # self.driverController.y().whileTrue(
        #     self.driveSubsystem.driveAutoPath("Barge to Processor")
        # )
        # self.driverController.a().whileTrue(
        #     self.driveSubsystem.driveAutoPath("New Path")
        # )

        # self.driverController.rightBumper().onTrue(
        #     self.intake.move_intake(self.intake.Position.DEPLOYED)
        # )
        # self.driverController.leftBumper().onTrue(
        #     self.intake.move_intake(self.intake.Position.HOME).andThen(
        #         self.shoulder.move(self.shoulder.Position.READY).andThen(
        #             self.intake.stop_intake()
        #         )a
        #     )
        # )
        self.driverController.x().onTrue(self.move_to_position())
        self.driverController.b().onTrue(self.score())
        # self.driverController.rightBumper().onTrue(self.grab_coral_from_trough())
        self.driverController.povUp().onTrue(self.hold_coral_during_travel())
        self.driverController.povDown().onTrue(self.hold_algae_during_travel())
        self.driverController.start().onTrue(self.move_to_home())
        self.driverController.leftBumper().onTrue(self.move_to_station())
        self.driverController.a().onTrue(
            self.grab_coral_from_trough()
            # self.driveSubsystem.driveAutoPath("New Path")
        )
        self.driverController.leftStick().whileTrue(
            ManualRobotOrientedDrive(self.driverController, self.driveSubsystem)
        )

    def configureOperatorControls(self):
        """Configures triggers for manual control by tactical"""
        # wpilib.SmartDashboard.putData("Deploy Intake", self.intake.deploy_)
        # wpilib.SmartDashboard.putData("Retract Intake",
        # self.intake.retract())
        self.tacticalController.povRight().onTrue(self.set_scoring_config(algae_L34))
        self.tacticalController.povDown().onTrue(
            self.set_scoring_config(algae_L23)
        )  # self.shoulder.move(-0.25))
        self.tacticalController.povLeft().onTrue(
            self.set_scoring_config(L3_dunk)
        )  # self.shoulder.move(0))
        self.tacticalController.povUp().onTrue(
            self.set_scoring_config(L4_dunk)
        )  # self.shoulder.move(0.125))
        self.tacticalController.leftBumper().onTrue(
            self.set_scoring_config(algae_process)
        )
        self.tacticalController.rightBumper().onTrue(
            self.elevator.move(3)
            .alongWith(self.shoulder.move(0))
            .andThen(self.climber.deploy_climber(1.7))
        )
        self.tacticalController.start().onTrue(self.climber.run_motor())
        self.tacticalController.a().onTrue(
            self.set_scoring_config(L1_shoot)
            # self.shoulder.move(self.shoulder.Position.LEVEL1)
        )
        self.tacticalController.b().onTrue(
            self.set_scoring_config(L2_shoot)
            # self.shoulder.move(self.shoulder.Position.LEVEL2).andThen(
            #     self.arm.move(self.arm.Position.CORAL_L2_PLACE)
            # )
        )
        self.tacticalController.x().onTrue(
            self.set_scoring_config(L3_shootV2)
            # self.shoulder.move(self.shoulder.Position.LEVEL3)
        )
        self.tacticalController.y().onTrue(
            self.set_scoring_config(L4_shoot)
            # self.shoulder.move(self.shoulder.Position.LEVEL4).andThen(
            #     self.arm.move(self.arm.Position.CORAL_L4_PLACE)
            # )
        )

    def configureSysIDButtonBindings(self):
        # Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        # once.
        self.driverController.a().whileTrue(
            self.driveSubsystem.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward)
        )
        self.driverController.b().whileTrue(
            self.driveSubsystem.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse)
        )
        self.driverController.x().whileTrue(
            self.driveSubsystem.sysIdDynamicDrive(SysIdRoutine.Direction.kForward)
        )
        self.driverController.y().whileTrue(
            self.driveSubsystem.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse)
        )

    def getAutonomousCommand(self):
        return self.autochooser.getSelected()
