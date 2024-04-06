package org.frc1410.crescendo2024.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import static org.frc1410.crescendo2024.util.Constants.*;

public class Camera implements Subsystem {
	private final PhotonCamera photonCamera = new PhotonCamera(CAMERA_NAME);

	private final PhotonPoseEstimator photonPoseEstimator;

	private final StructPublisher<Pose2d> visionOnlyPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("visionOnlyPose", Pose2d.struct).publish();

	public Camera() {
		AprilTagFieldLayout layout;

		// layout = new AprilTagFieldLayout(List.of(new AprilTag(4, new Pose3d(16.579342,  5.547867999999999, 1.4511020000000001, new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))))), 16.541, 8.211);

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
				System.out.println("Pose is present");
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
