// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.SUB_Vision;
import org.littletonrobotics.junction.AutoLog;

public interface IO_SwerveBase {

	@AutoLog
	public static class SwerveInputs {
		public Pose2d pose = new Pose2d();
		public Rotation2d yaw = new Rotation2d();
		public Rotation2d odometryHeading = new Rotation2d();
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	void updateInputs(SwerveInputs inputs);

	void drive(
			Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop);

	void drive(
			Translation2d translation2d,
			double headingX,
			double headingY,
			double headingRads,
			double maxVelocity);

	void drive(Translation2d translation, Rotation2d angle);

	double getMaximumVelocity();

	double getMaximumAngularVelocity();

	void updateEstimations(SUB_Vision vision);

	void updateOdometry();

	void resetOdometry(Pose2d pose);

	Pose2d getPose();

	Rotation2d getHeading();

	ChassisSpeeds getRobotVelocity();

	void setChassisSpeeds(ChassisSpeeds chassisSpeeds);

	com.pathplanner.lib.config.PIDConstants getHeadingPID();

	double getConfigurationRadius();

	void setBrakeMode(boolean isBrakeMode);

	void setZero();

	void setLock();

	void zeroGyro();

	void setDriveMotorCurrentLimit(int currentLimit);

	void setTurnMotorCurrentLimit(int currentLimit);

	public Rotation2d getYaw();
}
