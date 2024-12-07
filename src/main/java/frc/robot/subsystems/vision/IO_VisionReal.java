// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.Camera;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class IO_VisionReal implements IO_VisionBase {

	// Initialize cameras
	PhotonCamera forwardCamera = new PhotonCamera("WomBBATForward");
	PhotonCamera rearCamera = new PhotonCamera("WomBBATRear");

	// Object detection camera
	// PhotonCamera lifecam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

	double lastEstTimestamp = 0.0;

	// Initialize AprilTag field layout
	AprilTagFieldLayout fieldLayout;

	// Initialize pose estimators
	PhotonPoseEstimator forwardPoseEstimator;
	PhotonPoseEstimator rearPoseEstimator;

	List<PhotonTrackedTarget> targets;

	public IO_VisionReal() {

		targets = new ArrayList<>();

		// Disable driver mode for the cameras
		forwardCamera.setDriverMode(false);
		rearCamera.setDriverMode(false);

		// Try loading AprilTag field layout

		try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			DriverStation.reportError("Failed to load AprilTag field layout!", false);
		}

		// Initialize pose estimators
		forwardPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				Constants.Vision.Camera.FORWARD_CAMERA.ROBOT_TO_CAMERA);

		forwardPoseEstimator = new PhotonPoseEstimator(
				fieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				Constants.Vision.Camera.FORWARD_CAMERA.ROBOT_TO_CAMERA);
		rearPoseEstimator = new PhotonPoseEstimator(
				fieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				Constants.Vision.Camera.REAR_CAMERA.ROBOT_TO_CAMERA);

		// Set fallback strategies
		forwardPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		rearPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	@Override
	public void updateInputs(VisionInputs inputs) {
		inputs.forwardCameraIsOn = forwardCamera.isConnected();
		inputs.rearCameraIsOn = rearCamera.isConnected();

		inputs.forwardCameraHasTargets = forwardCamera.getLatestResult().hasTargets();
		inputs.rearCameraHasTargets = rearCamera.getLatestResult().hasTargets();

		inputs.forwardCameraLatency = forwardCamera.getLatestResult().getTimestampSeconds();
		inputs.rearCameraLatency = rearCamera.getLatestResult().getTimestampSeconds();
	}

	/**
	 * Get the pose of the donut based on the latest result from the lifecam and
	 * trignometry.
	 *
	 * @return The pose of the donut
	 *         <p>
	 *         public Pose2d getDonutPose() { if
	 *         (lifecam.getLatestResult().hasTargets()) {
	 *         <p>
	 *         // PhotonUtils.estimateCameraToTargetTranslation(targetDistMeters,
	 *         targetYaw)
	 *         <p>
	 *         double targetYaw =
	 *         lifecam.getLatestResult().getBestTarget().getYaw();
	 *         <p>
	 *         double targetPitch =
	 *         lifecam.getLatestResult().getBestTarget().getPitch();
	 *         <p>
	 *         double cameraPitchAngle = 7.0; double cameraHeightMeters = 0.3;
	 *         <p>
	 *         double rDistanceMeters = 1.0 /
	 *         Math.tan(Math.toRadians(cameraPitchAngle) +
	 *         Math.toRadians(targetPitch)) cameraHeightMeters;
	 *         <p>
	 *         double xDistanceMeters = Math.cos(Math.toRadians(targetYaw)) *
	 *         rDistanceMeters; double
	 *         yDistanceMeters = Math.sin(Math.toRadians(targetYaw)) *
	 *         rDistanceMeters;
	 *         <p>
	 *         return new Pose2d(xDistanceMeters, yDistanceMeters, new
	 *         Rotation2d()); } else { return
	 *         new Pose2d(-1, -1, new Rotation2d()); } }
	 */

	/**
	 * Returns the latest result based on the given Camera.
	 *
	 * @param camera The camera to get the result from
	 * @return The latest result
	 */
	@Override
	public PhotonPipelineResult getResult(Camera camera) {
		if (camera == Constants.Vision.Camera.FORWARD_CAMERA) {
			return forwardCamera.getLatestResult();
		} else {
			return rearCamera.getLatestResult();
		}
	}

	/**
	 * Retrieves the list of tracked targets from the specified camera.
	 *
	 * @param camera The camera object from which to retrieve the targets
	 * @return The list of tracked targets from the camera
	 */
	@Override
	public List<PhotonTrackedTarget> getTargets(Camera camera) {

		targets.clear();

		var latestResult = getResult(camera);

		if (latestResult.hasTargets()) {
			// Add camera targets to list
			for (PhotonTrackedTarget target : latestResult.getTargets()) {
				targets.add(target);
			}
			return targets;
		}
		return targets;
	}

	/**
	 * Returns the pose estimator for the specified camera.
	 *
	 * @param camera The camera to get the pose estimator for
	 * @return The pose estimator for the specified camera
	 */
	@Override
	public PhotonPoseEstimator getPoseEstimator(Camera camera) {
		if (camera == Constants.Vision.Camera.FORWARD_CAMERA) {
			return forwardPoseEstimator;
		} else {
			return rearPoseEstimator;
		}
	}

	/**
	 * Retrieves the estimated global pose from the specified camera.
	 *
	 * @param camera The camera from which to retrieve the estimated pose
	 * @return An optional containing the estimated robot pose, or empty if the
	 *         camera is not valid
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {

		if (camera == Camera.FORWARD_CAMERA) {
			Optional<EstimatedRobotPose> visionEst = forwardPoseEstimator.update(forwardCamera.getLatestResult());
			double latestTimestamp = forwardCamera.getLatestResult().getTimestampSeconds();
			boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

			if (newResult)
				lastEstTimestamp = latestTimestamp;
			return visionEst;

		} else if (camera == Camera.REAR_CAMERA) {
			Optional<EstimatedRobotPose> visionEst = rearPoseEstimator.update(rearCamera.getLatestResult());
			double latestTimestamp = rearCamera.getLatestResult().getTimestampSeconds();
			boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

			if (newResult)
				lastEstTimestamp = latestTimestamp;
			return visionEst;
		} else { // Limelight
			return null;
		}
	}

	/**
	 * Calculate the estimation standard deviations based on the estimated pose.
	 *
	 * @param estimatedPose The estimated pose for calculation
	 * @param camera        The camera to calculate the standard deviations for
	 * @return The calculated standard deviations
	 */
	public Matrix<N3, N1> getEstimationStdDevs(
			Pose2d estimatedPose, Camera camera, List<PhotonTrackedTarget> targets) {
		// Initialize standard deviations with the default values for a single tag
		var estStdDevs = Vision.SINGLE_TAG_STD_DEVS;

		int numTags = 0;
		double avgDist = 0;

		// Iterate over each target to calculate the average distance from the estimated
		// pose
		for (var tgt : targets) {
			// Get the pose of the tag from the field tags
			var tagPose = rearPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
			if (camera == Constants.Vision.Camera.FORWARD_CAMERA) {
				tagPose = forwardPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
			}

			// if (camera == Constants.Vision.Camera.LIMELIGHT) {
			// tagPose =
			// limelightPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
			// }

			if (tagPose.isEmpty())
				continue;
			numTags++;
			// Add the distance from the estimated pose to the tag's pose to the average
			// distance
			avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
		}

		// If no tags were detected, return the default standard deviations
		if (numTags == 0)
			return estStdDevs;

		// Calculate the average distance
		avgDist /= numTags;

		// If multiple tags are detected, use the standard deviations for multiple tags
		if (numTags > 1)
			estStdDevs = Vision.MULTI_TAG_STD_DEVS;

		// If only one tag is detected and the average distance is greater than 4
		// meters, set high
		// standard deviations
		if (numTags == 1 && avgDist > 4) {
			estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			// Otherwise, scale the standard deviations based on the square of the average
			// distance
			estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
		}

		// Return the calculated standard deviations
		return estStdDevs;
	}

	@Override
	public void setPose(Pose2d pose) {
	}

	@Override
	public PhotonTrackedTarget getTarget(Camera camera, int id) {
		if (camera == Constants.Vision.Camera.FORWARD_CAMERA) {
			if (forwardCamera.getLatestResult().hasTargets()) {
				for (var trgt : forwardCamera.getLatestResult().getTargets())
					if (trgt.getFiducialId() == id)
						return trgt;
				return null;
			} else
				return null;
		}
		return null;
	}
}
