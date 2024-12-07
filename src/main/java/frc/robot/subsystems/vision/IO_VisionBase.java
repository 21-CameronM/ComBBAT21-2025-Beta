// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.Vision.Camera;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface IO_VisionBase {

	@AutoLog
	public static class VisionInputs {

		public boolean forwardCameraIsOn = false;
		public boolean rearCameraIsOn = false;

		public boolean forwardCameraHasTargets = false;
		public boolean rearCameraHasTargets = false;

		public double forwardCameraLatency = 0.0;
		public double rearCameraLatency = 0.0;
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	void updateInputs(VisionInputs inputs);

	public PhotonTrackedTarget getTarget(Camera camera, int id);

	public void setPose(Pose2d pose);

	public PhotonPipelineResult getResult(Camera camera);

	public List<PhotonTrackedTarget> getTargets(Camera camera);

	public PhotonPoseEstimator getPoseEstimator(Camera camera);

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera);

	public Matrix<N3, N1> getEstimationStdDevs(
			Pose2d estimatedPose, Camera camera, List<PhotonTrackedTarget> targets);
}
