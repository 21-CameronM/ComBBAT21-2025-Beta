// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.IO_SwerveReal;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.IO_VisionReal;
import frc.robot.subsystems.vision.IO_VisionSim;
import frc.robot.subsystems.vision.SUB_Vision;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

	private final CommandXboxController driverController = new CommandXboxController(0);

	//private final SUB_Vision vision;
	// private final AddressableLedStrip led = new AddressableLedStrip(1, 64);

	private final SUB_Swerve swerve;

	// private final AutoSelector autoSelector;

	public RobotContainer() {
		//vision = new SUB_Vision(Robot.isSimulation() ? new IO_VisionSim() : new IO_VisionReal());
		//swerve = new SUB_Swerve(new IO_SwerveReal(), vision, driverController);
		swerve = new SUB_Swerve(new IO_SwerveReal(), null, driverController);

		// autoSelector = new AutoSelector();
		configureDefaultCommands();
		logMetadata();
	}

	private void configureDefaultCommands() {

		/*
		 * swerve.setDefaultCommand(
		 * swerve.drive(
		 * () -> driverController.getRawAxis(1) * 0.2,
		 * () -> driverController.getRawAxis(0) * 0.2,
		 * () -> driverController.getRawAxis(3)));
		 */
		swerve.setDefaultCommand(
				swerve.drive(
						() -> -driverController.getRawAxis(1) * 0.5,
						() -> -driverController.getRawAxis(0) * 0.5,
						() -> -driverController.getRawAxis(4),
						() -> -driverController.getRawAxis(5),
						() -> driverController.a().getAsBoolean(),
						null));

		driverController.y().onTrue(Commands.runOnce(() -> swerve.zeroGyro()));
	}

	public void logMetadata() {
		Logger.recordMetadata("Event Name", DriverStation.getEventName());
		Logger.recordMetadata("Driver Station Location", DriverStation.getLocation() + "");
		Logger.recordMetadata("Match Number", DriverStation.getMatchNumber() + "");
		Logger.recordMetadata("Match Type", DriverStation.getMatchType() + "");
		Logger.recordMetadata("Replay Number", DriverStation.getReplayNumber() + "");
		Logger.recordMetadata("Robot Mode", "" + Constants.CURRENT_MODE);
	}

	/**
	 * Get the autonomous command.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		return new PathPlannerAuto("TestAuto");
	}
}
