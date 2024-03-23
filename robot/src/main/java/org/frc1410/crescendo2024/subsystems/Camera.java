package org.frc1410.crescendo2024.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.Optional;

import static org.frc1410.crescendo2024.util.Constants.*;

public class Camera implements Subsystem {
	private final PhotonCamera photonCamera = new PhotonCamera(CAMERA_NAME);

	private final PhotonPoseEstimator photonPoseEstimator;

	private final StructPublisher<Pose2d> visionOnlyPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("visionOnlyPose", Pose2d.struct).publish();

	public Camera() {
		AprilTagFieldLayout layout;

		try {
			layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch(IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			layout = null;
		}

		this.photonPoseEstimator = new PhotonPoseEstimator(
			layout,
			PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			this.photonCamera,
			CAMERA_POSE
		);
	}

	public Optional<EstimatedRobotPose> getEstimatedPose() {
		if(this.photonCamera.getLatestResult().hasTargets()) {
			var pose = this.photonPoseEstimator.update();

			if (pose.isPresent()) {
				this.visionOnlyPosePublisher.set(pose.get().estimatedPose.toPose2d());
			} else {
				this.visionOnlyPosePublisher.set(null);
			}

			return pose;
		} else {
			return Optional.empty();
		}
	}
}
