#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import os
import wpilib
from wpilib import DriverStation
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
    robotToCamera,
    robotToCamera2,
)
from commands2.cmd import runOnce, startEnd, waitUntil
from commands2 import DeferredCommand, PrintCommand, RepeatCommand, InterruptionBehavior
from commands2.sysid import SysIdRoutine

from FROGlib.xbox import FROGXboxDriver, FROGXboxTactical

from subsystems.drivechassis import DriveChassis
from subsystems.positioning import Position
from subsystems.vision import VisionPose
from subsystems.lift import Lift
from subsystems.shoulder import Shoulder
from subsystems.grabber import Grabber
from subsystems.arm import Arm
from subsystems.Intake import Intake
from subsystems.climber import Climber

from commands.drive.field_oriented import (
    ManualDrive,
)


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

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
        # self.camera1 = VisionPose(kCamera1Name, robotToCamera)
        # self.camera2 = VisionPose(kCamera2Name, robotToCamera2)

        # Sensors/Cameras
        # Add each positioning camera to the positioningCameras list
        # self.positioningCameras.append(self.camera1)
        # self.positioningCameras.append(self.camera2)
        self.positioningCameras = []

        self.positioning = Position()

        # Subsystems
        self.driveSubsystem = DriveChassis(self.positioningCameras)
        self.elevator = Lift()
        self.shoulder = Shoulder()
        self.arm = Arm()
        self.grabber = Grabber()
        self.intake = Intake()
        self.climber = Climber()

        self.registerNamedCommands()

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.driveSubsystem.setDefaultCommand(
            ManualDrive(self.driverController, self.driveSubsystem)
        )

    def registerNamedCommands(self):
        pass

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        self.configureDriverControls()
        self.configureOperatorControls()

    def configureTestBindings(self):
        self.elevator.setDefaultCommand(
            self.elevator.joystick_move_command(self.driverController.getLeftY)
        )
        self.shoulder.setDefaultCommand(
            self.shoulder.joystick_move_command(self.driverController.getRightY)
        )
        self.arm.setDefaultCommand(
            self.arm.joystick_move_command(self.driverController.getRightX)
        )

        self.driverController.back().onTrue(self.elevator.home())
        self.driverController.a().onTrue(self.grabber.intake_algae())
        self.driverController.b().onTrue(self.grabber.intake_coral())

    def configureSysIDButtonBindings(self):
        # Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        # once.
        pass

    def configureHomeRoutines(self):
        wpilib.SmartDashboard.putData("Home Elevator", self.elevator.home())
        wpilib.SmartDashboard.putData("Home Arm", self.arm.set_home())
        wpilib.SmartDashboard.putData("Home Intake", self.intake.set_home())

    def configureDriverControls(self):
        """DRIVER CONTROLS"""
        # self.driverController.start().onTrue(
        #     runOnce(lambda: self.driveSubsystem.setFieldPositionFromVision())
        # )
        self.driverController.rightBumper().onTrue(self.grabber.intake_coral())
        self.driverController.rightBumper().onFalse(self.grabber.eject_coral())
        self.driverController.leftBumper().onTrue(
            runOnce(self.intake.run_intake()).alongWith(
                self.intake.move(self.intake.Position.DEPLOYED)
            )
        )
        self.driverController.leftBumper().onFalse(
            runOnce(self.intake.stop_intake()).alongWith(
                self.intake.move(self.intake.Position.HOME)
            )
        )

    def configureOperatorControls(self):
        """OPERATOR CONTROLS"""
        # wpilib.SmartDashboard.putData("Deploy Intake", self.intake.deploy_)
        # wpilib.SmartDashboard.putData("Retract Intake",
        # self.intake.retract())
        self.tacticalController.povDown().onTrue(self.shoulder.move(-0.25))
        self.tacticalController.povLeft().onTrue(self.shoulder.move(0))
        self.tacticalController.povUp().onTrue(self.shoulder.move(0.125))
        self.tacticalController.leftBumper().onTrue(
            self.arm.move(self.arm.Position.CORAL_PICKUP)
        )
        self.tacticalController.leftBumper().onFalse(
            self.arm.move(self.arm.Position.RETRACTED)
        )
        self.tacticalController.a().onTrue(
            self.shoulder.move(self.shoulder.Position.LEVEL1)
        )
        self.tacticalController.b().onTrue(
            self.shoulder.move(self.shoulder.Position.LEVEL2).andThen(
                self.arm.move(self.arm.Position.CORAL_L2_PLACE)
            )
        )
        self.tacticalController.x().onTrue(
            self.shoulder.move(self.shoulder.Position.LEVEL3)
        )
        self.tacticalController.y().onTrue(
            self.shoulder.move(self.shoulder.Position.LEVEL4).andThen(
                self.arm.move(self.arm.Position.CORAL_L4_PLACE)
            )
        )

    def getAutonomousCommand(self):

        # return self.autochooser.getSelected()
        pass
