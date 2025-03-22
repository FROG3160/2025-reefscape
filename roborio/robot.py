#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing
from phoenix6 import SignalLogger
import wpilib
import commands2
from commands2.cmd import runOnce
from wpimath.geometry import Pose2d
from wpilib import DataLogManager, DriverStation
from wpilib import RobotController

from robotcontainer import RobotContainer

# Temporary falcon motor control
from phoenix6.controls import VelocityDutyCycle, VelocityVoltage


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        # Start recording to log
        DataLogManager.start()
        # start logging DS and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())
        # Set log path for ctre on the first USB drive found
        # TODO: https://github.com/FROG3160/2024-crescendo/issues/162 this doesn't work, check the Roborio to see if USB
        #   is really on /media/sda1
        # SignalLogger.set_path("/media/sda1/ctre-logs/")
        # start SignalLogger if the robot is running it
        if self.isReal():
            SignalLogger.start()

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

        self.startingPose2d = Pose2d(0, 0, 0)

        wpilib.SmartDashboard.putData("DriveTrain", self.container.driveSubsystem)

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.container.positioning.setReefTags(DriverStation.getAlliance())
        self.container.move_off_line()
        # self.autonomousCommand = self.container.getAutonomousCommand()
        # if self.autonomousCommand:
        #     # if self.isReal():
        #     #     self.container.elevationSubsystem.homeShooterCommand().andThen(
        #     #         self.autonomousCommand
        #     #     ).withName(self.autonomousCommand.getName()).schedule()
        #     # else:
        #     #     self.autonomousCommand.schedule()

        #     self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # set's the driver X, Y output based on which alliance we are on
        self.container.driverController.set_alliance(DriverStation.getAlliance())
        self.container.positioning.setReefTags(DriverStation.getAlliance())
        self.container.home_subsystems()

        # make sure the drive is enabled
        self.container.driveSubsystem.enable()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.home_subsystems()
        self.container.configureTestBindings()
        # self.container.configureSysIDButtonBindings()
