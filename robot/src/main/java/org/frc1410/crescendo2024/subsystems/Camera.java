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

	private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

	private LEDs leds = new LEDs();

	private AprilTagFieldLayout layout;

	private PhotonPoseEstimator photonPoseEstimator;

	public Camera() {

		try {
			layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
			var alliance = DriverStation.getAlliance();
				// layout.setOrigin(alliance == Optional.of(Alliance.Blue) ?
				// 	OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
		} catch(IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			layout = null;
		}

		photonPoseEstimator = new PhotonPoseEstimator(
			layout,
			PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			camera,
			CAMERA_POSE
		);
	}

	public Optional<EstimatedRobotPose> getEstimatedPose() {
		if(camera.getLatestResult().hasTargets()) {
			return photonPoseEstimator.update();
		} else {
			return Optional.empty();
		}
	}
}
