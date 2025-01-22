#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import os
import wpilib
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
)
from commands2.cmd import runOnce, startEnd, waitUntil
from commands2 import DeferredCommand, PrintCommand, RepeatCommand, InterruptionBehavior
from commands2.sysid import SysIdRoutine

from FROGlib.xbox import FROGXboxDriver, FROGXboxTactical

from subsystems.drivechassis import DriveChassis
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
        # self.operatorController = FROGXboxOperator(kOperatorControllerPort, kDeadband)

        # Create all subsystems here.  If a subsystem is needed by other subsystems, create it first,
        # then pass it in to the subsystems needing it.
        self.driveSubsystem = DriveChassis()

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

    def configureSysIDButtonBindings(self):
        # Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        # once.
        self.driverController.a().whileTrue(
            self.driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        self.driverController.b().whileTrue(
            self.driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        self.driverController.x().whileTrue(
            self.driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        self.driverController.y().whileTrue(
            self.driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

        wpilib.SmartDashboard.putData(
            "Quasistatic Forward",
            self.driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
        )
        wpilib.SmartDashboard.putData(
            "Quasistatic Reverse",
            self.driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
        )
        wpilib.SmartDashboard.putData(
            "Dynamic Forward",
            self.driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward),
        )
        wpilib.SmartDashboard.putData(
            "Dynamic Reverse",
            self.driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse),
        )

    def configureDriverControls(self):
        """DRIVER CONTROLS"""
        # self.driverController.start().onTrue(
        #     runOnce(lambda: self.driveSubsystem.setFieldPositionFromVision())
        # )

    def configureOperatorControls(self):
        """OPERATOR CONTROLS"""
        pass

    def getAutonomousCommand(self):

        # return self.autochooser.getSelected()
        pass
