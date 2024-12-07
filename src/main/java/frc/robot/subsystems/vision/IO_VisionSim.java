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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Vision.Camera;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class IO_VisionSim implements IO_VisionBase {

	// Initialize visionSim
	VisionSystemSim visionSim = new VisionSystemSim("main");

	// Initialize cameras

	PhotonCamera forwardCamera = new PhotonCamera("WomBBATForward");
	PhotonCamera rearCamera = new PhotonCamera("WomBBATRear");

	PhotonCameraSim simForwardCamera;
	PhotonCameraSim simRearCamera;

	// Object detection camera
	// PhotonCamera lifecam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

	double lastEstTimestamp = 0.0;

	// Initialize AprilTag field layout
	AprilTagFieldLayout fieldLayout;

	List<PhotonTrackedTarget> targets;

	public IO_VisionSim() {

		// Try loading AprilTag field layout
		try {
			fieldLayout =
					AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			DriverStation.reportError("Failed to load AprilTag field layout!", false);
		}
		visionSim.addAprilTags(fieldLayout);

		simForwardCamera = new PhotonCameraSim(forwardCamera);
		simRearCamera = new PhotonCameraSim(rearCamera);
		visionSim.addCamera(simForwardCamera, Constants.Vision.Camera.FORWARD_CAMERA.ROBOT_TO_CAMERA);
		visionSim.addCamera(simRearCamera, Constants.Vision.Camera.REAR_CAMERA.ROBOT_TO_CAMERA);
		simForwardCamera.enableRawStream(true);
		simForwardCamera.enableProcessedStream(true);
		simForwardCamera.enableDrawWireframe(true);
	}

	@Override
	public void updateInputs(VisionInputs inputs) {
		inputs.forwardCameraIsOn = true;
		inputs.rearCameraIsOn = true;

		inputs.forwardCameraHasTargets = false;
		inputs.rearCameraHasTargets = false;

		inputs.forwardCameraLatency = forwardCamera.getLatestResult().getTimestampSeconds();
		inputs.rearCameraLatency = rearCamera.getLatestResult().getTimestampSeconds();
	}

	@Override
	public PhotonPipelineResult getResult(Camera camera) {
		return null;
	}

	@Override
	public List<PhotonTrackedTarget> getTargets(Camera camera) {
		return null;
	}

	@Override
	public PhotonPoseEstimator getPoseEstimator(Camera camera) {
		return null;
	}

	@Override
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {
		return null;
	}

	@Override
	public Matrix<N3, N1> getEstimationStdDevs(
			Pose2d estimatedPose, Camera camera, List<PhotonTrackedTarget> targets) {
		return null;
	}

	@Override
	public void setPose(Pose2d pose) {
		visionSim.update(pose);
	}

	@Override
	public PhotonTrackedTarget getTarget(Camera camera, int id) {
		if (camera == Constants.Vision.Camera.FORWARD_CAMERA) {
			if (forwardCamera.getLatestResult().hasTargets()) {
				for (var trgt : forwardCamera.getLatestResult().getTargets())
					if (trgt.getFiducialId() == id) return trgt;
				return null;
			} else return null;
		}
		return null;
	}
}
