package org.frc1410.crescendo2024.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.Optional;

import static org.frc1410.crescendo2024.util.Constants.*;

public class Camera implements Subsystem {

	private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);

	private AprilTagFieldLayout layout;

	private PhotonPoseEstimator photonPoseEstimator;

	public Camera() {

		try {
			layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
			var alliance = DriverStation.getAlliance();
//             layout.setOrigin(alliance == Optional.of(Alliance.Blue) ?
//                   OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
		} catch(IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			layout = null;
		}

		photonPoseEstimator = new PhotonPoseEstimator(
			layout,
			PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			camera,
			new Transform3d()
		);
	}

	public Optional<EstimatedRobotPose> getEstimatedPose() {
		return photonPoseEstimator.update();
	}

	public PhotonPipelineResult getLatestResult() {
		return camera.getLatestResult();
	}
}
