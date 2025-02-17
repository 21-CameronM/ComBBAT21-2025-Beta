// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Auto.ScoringPoses;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

/**
 * The MathCalc class provides functionality to find the best RPM and angle for a projectile to hit
 * a target based on the horizontal distance and other factors. This class also features a few other
 * random methods that may be of use.
 */
public class MathCalc {

	/**
	 * Calculate the angle required for hitting the target based a exponential equation and the
	 * distance to the target
	 *
	 * @param distanceToTarget The distance to the target
	 * @return The calculated required angle to hit the target.
	 * @units RPM & DEG
	 */
	private static double calculateInterpolate(double distanceToTarget) {
		// return (50.5739 * Math.exp(-distanceToTarget)) + 36.3442;

		return (61.9759 * Math.exp(-0.85144 * distanceToTarget)) + 25.5309 + 0.35;

		/*
		if (distanceToTarget > 3.3) {
			return ((61.9759 * Math.exp(-0.49144 * distanceToTarget)) + 25.5309 + 0.35) // Old: -0.49144
					- Constants.Maestro.ARM_OFFSET_DEGREES;

		} else { // Less than 3.3 meters
			return ((61.9759 * Math.exp(-0.49144 * distanceToTarget)) + 25.5309 + 0.35)
					- Constants.Maestro.ARM_OFFSET_DEGREES;
		}
		*/

		/*
		return 39.3257 * Erf.erf(1.20504 * distanceToTarget)
				+ 95.1292 * Math.exp(-1.30193 * distanceToTarget);
				*/
		// Lower arm angle higher exponet
	}

	public static double calculateArmAngle(Pose2d robotPose) {

		var alliance = DriverStation.getAlliance();
		Pose2d targetPose;

		if (alliance.get() == Alliance.Blue) {
			targetPose = ScoringPoses.BLU_SPEAKER.pose;

		} else if (alliance.get() == Alliance.Red) {
			targetPose = ScoringPoses.RED_SPEAKER.pose;

		} else {
			targetPose = new Pose2d();
			DriverStation.reportError(
					"[error] [calculateArmAngle] Could not find alliance for auto angle", false);
		}

		double distanceToSpeaker = PhotonUtils.getDistanceToPose(robotPose, targetPose);

		double armCalculation = MathCalc.calculateInterpolate(distanceToSpeaker);

		Logger.recordOutput("[calculateArmAngle] Dynamic Angle", armCalculation);

		return armCalculation;
	}

	/**
	 * Calculate the RPM required for hitting the target based on the current angle and distance to
	 * the target https://www.desmos.com/calculator/vu2iw2ssbj
	 * https://www.desmos.com/calculator/on4xzwtdwz
	 *
	 * @param currentAngle The current angle of the projectile motion
	 * @param distanceToTarget The distance to the target
	 * @return The calculated required RPM to hit the target.
	 * @author Dustin B
	 * @units RPM & DEG
	 */
	public static double physicsCalculate(double currentAngle, double distanceToTarget) {

		double g = -1 / 2 * 9.8 * Math.pow(distanceToTarget, 2);

		double den1 = Math.pow(Math.PI * 2 * 0.05 * 1 / 60, 2) * Math.pow(Math.cos(currentAngle), 2);

		double den2 =
				2.2 - (0.45 + 0.47 * Math.sin(currentAngle)) - distanceToTarget * Math.tan(currentAngle);

		double temp = g / den1;

		temp = temp / den2;

		return Math.sqrt(temp);
	}

	/**
	 * Generates a random integer between the min and max
	 *
	 * @param min The minimum value
	 * @param max The maximum value
	 * @return Random number
	 */
	public static int random(int min, int max) {
		return (int) (Math.random() * ((max - min) + 1)) + min;
	}

	/**
	 * Calculate the area of a triangle.
	 *
	 * @param x1, y1 coordinates of the first point
	 * @param x2, y2 coordinates of the second point
	 * @param x3, y3 coordinates of the third point
	 * @return The area of the triangle.
	 */
	public static double area(double x1, double y1, double x2, double y2, double x3, double y3) {
		return Math.abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
	}

	/**
	 * Check if a point is inside a triangle
	 *
	 * @param pointOne The first point
	 * @param pointTwo The second point
	 * @param pointThree The third point
	 * @param robotPoint The point to check
	 * @return true if the robot point is inside the triangle, false otherwise.
	 */
	public static boolean isPointInsideTriangle(
			Translation2d pointOne,
			Translation2d pointTwo,
			Translation2d pointThree,
			Translation2d robotPoint) {
		// Calculate area of triangle ABC
		double A =
				area(
						pointOne.getX(),
						pointOne.getY(),
						pointTwo.getX(),
						pointTwo.getY(),
						pointThree.getX(),
						pointThree.getY());

		// Calculate area of triangle PBC
		double A1 =
				area(
						robotPoint.getX(),
						robotPoint.getY(),
						pointTwo.getX(),
						pointTwo.getY(),
						pointThree.getX(),
						pointThree.getY());

		// Calculate area of triangle PAC
		double A2 =
				area(
						pointOne.getX(),
						pointOne.getY(),
						robotPoint.getX(),
						robotPoint.getY(),
						pointThree.getX(),
						pointThree.getY());

		// Calculate area of triangle PAB
		double A3 =
				area(
						pointOne.getX(),
						pointOne.getY(),
						robotPoint.getX(),
						robotPoint.getY(),
						pointTwo.getX(),
						pointTwo.getY());

		// Check if sum of A1, A2 and A3 is same as A
		return A == A1 + A2 + A3;
	}
}
