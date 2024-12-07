// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** This class contains constants used in the robot code. */
public class Constants {

	// Current robot mode and loop period.
	public static final RobotMode CURRENT_MODE = RobotMode.REAL;

	// Use extreme caution when enabled, manually controls PID setpoints through
	// smart dashboard.
	public static final boolean PID_TEST_MODE = false;

	// Enables auto driving to a pose with pathplanner during teletop. Be very
	// careful when activated!
	public static final boolean TELEOP_AUTO_DRIVE_ENABLED = false;

	// Enables vision based odometry for swerve drive.
	public static final boolean SWERVE_VISION_ODOMETRY_ENABLED = false;

	// Enables snap angle PID for swerve drive.
	public static final boolean TELEOP_DRIVER_SNAP_ENABLED = false;

	// A non-static variable that determines the rotation override state of the
	// swerve.
	public static RotationOverrideState ROTATION_OVERRIDE_STATE = RotationOverrideState.OFF;

	public enum RotationOverrideState {
		OFF,
		SPEAKER,
		AMP
	}

	// Current robot mode
	public enum RobotMode {
		REAL, // Running on a real robot
		SIM, // Running a physics simulator
		REPLAY // Replaying from a log file
	}

	public static class Vision {
		/*
		 * The standard deviations of our vision estimated poses, which affect
		 * correction rate
		 * (Fake values. Experiment and determine estimation noise on an actual robot.)
		 * X, Y, Theta
		 * public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4,
		 * 4, 8);
		 * public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5,
		 * 0.5, 1);
		 */
		public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 4);
		public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.8, 0.8, 1.25);

		// Minimum ambiguity for a tag to be used for vision basced odometry
		public static final double MIN_AMBUGUITY = 0.5;

		// Vision warning trigger threshold in miliseconds
		public static final double LATENCY_THRESHOLD_MILI_SEC = 45.0;

		/*
		 * Robot Space
		 * 3d Cartesian Coordinate System with (0,0,0) located at the center of the
		 * robot’s frame projected down to the floor.
		 * X+ → Pointing forward (Forward Vector)
		 * Y+ → Pointing toward the robot’s right (Right Vector)
		 * Z+ → Pointing upward (Up Vector)
		 */

		// Camera enum, contains name and position of the camera relative to the robot.
		public enum Camera {
			FORWARD_CAMERA(
					"WomBBATForward",
					"WomBBATForward",
					new Transform3d(
							new Translation3d(
									Units.inchesToMeters(6.329),
									Units.inchesToMeters(0),
									Units.inchesToMeters(5.468)),
							new Rotation3d(
									Units.degreesToRadians(0),
									Units.degreesToRadians(-30),
									Units.degreesToRadians(0)))),
			REAR_CAMERA(
					"WomBBATRear",
					"WomBBATRear",
					new Transform3d(
							new Translation3d(
									Units.inchesToMeters(13.814),
									Units.inchesToMeters(0),
									Units.inchesToMeters(5.468)),
							new Rotation3d(
									Units.degreesToRadians(0),
									Units.degreesToRadians(-30),
									Units.degreesToRadians(180))));

			public final String PHOTON_NAME;
			public final String CAMERA_NAME;

			public final Transform3d ROBOT_TO_CAMERA;

			Camera(String photonName, String cameraName, Transform3d robotToCamera) {
				this.PHOTON_NAME = photonName;
				this.CAMERA_NAME = cameraName;
				this.ROBOT_TO_CAMERA = robotToCamera;
			}
		}
	}

	public static class SubsystemConst {
		public static final double MAX_MODULE_SPEED_MPS = Units.feetToMeters(15.1);
	}

	public static class Auto {

		// Swerve align PID
		public static final com.pathplanner.lib.config.PIDConstants SWERVE_ALIGN_PID = new com.pathplanner.lib.config.PIDConstants(0.015, 0.0, 0.003);

		// Max LINEAR velocity and acceleration of the swerve drive during auto
		public static final double MAX_VELOCITY_MPS = 1.5;
		public static final double MAX_ACCELERATION_MPS_SQ = 2.0;

		// Max ANGULAR velocity and acceleration of the swerve drive during auto
		public static final double MAX_ANGULAR_VELOCITY_RADS = 360;
		public static final double MAX_ANGULAR_VELOCITY_RADS_SQ = 540;

		// Positions of the game elements on the field
		public enum ScoringPoses {
			BLU_SPEAKER(new Pose2d(0, 5.5, new Rotation2d())),
			RED_SPEAKER(new Pose2d(16.5, 5.5, new Rotation2d())),

			BLU_AMP(new Pose2d(1.85, 7.59, Rotation2d.fromDegrees(-90))),
			RED_AMP(new Pose2d(14.75, 7.59, Rotation2d.fromDegrees(90)));

			public final Pose2d pose;

			ScoringPoses(Pose2d pose) {
				this.pose = pose;
			}
		}

		// Positions of the notes / donuts on the field
		public enum NotePoses {
			BLU_TOP(new Pose2d(2.9, 7.0, Rotation2d.fromDegrees(0))),
			BLU_MIDDLE(new Pose2d(2.9, 5.5, Rotation2d.fromDegrees(0))),
			BLU_BOTTOM(new Pose2d(2.9, 4.1, Rotation2d.fromDegrees(0))),

			RED_TOP(new Pose2d(13.67, 7.0, Rotation2d.fromDegrees(0))),
			RED_MIDDLE(new Pose2d(13.67, 5.5, Rotation2d.fromDegrees(0))),
			RED_BOTTOM(new Pose2d(13.67, 4.1, Rotation2d.fromDegrees(0)));

			public final Pose2d pose;

			NotePoses(Pose2d pose) {
				this.pose = pose;
			}
		}

		// Where the robot should drive to on the field
		public enum DriveScoringPoseState {
			SPEAKER,
			AMP
		}

		// Zones of the field
		public enum FieldTriangles {
			BLU_LEFT_SPEAKER(
					new Translation2d(0, 6.5), new Translation2d(0.85, 6.0), new Translation2d(1.6, 6.8)),
			BLU_MIDDLE_SPEAKER_LEFT(
					new Translation2d(12.0, 5.5), new Translation2d(1.5, 5.5), new Translation2d(1.5, 5.5)),
			BLU_MIDDLE_SPEAKER_RIGHT(
					new Translation2d(12.0, 5.5), new Translation2d(12.0, 5.5), new Translation2d(1.5, 5.5)),
			BLU_RIGHT_SPEAKER(
					new Translation2d(0, 4.7), new Translation2d(0.85, 5.1), new Translation2d(1.6, 4.5));

			public final Translation2d pointOne;
			public final Translation2d pointTwo;
			public final Translation2d pointThree;

			FieldTriangles(Translation2d pointOne, Translation2d pointTwo, Translation2d pointThree) {
				this.pointOne = pointOne;
				this.pointTwo = pointTwo;
				this.pointThree = pointThree;
			}
		}
	}
}
