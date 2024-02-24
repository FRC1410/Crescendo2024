package org.frc1410.crescendo2024.util;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class ShootingPosition {

	public final Pose2d pose;
	public final double shooterRPM;
	public final double storageRPM;

	public ShootingPosition(Pose2d pose, double shooterRPM, double storageRPM) {
		this.pose = pose;
		this.shooterRPM = shooterRPM;
		this.storageRPM = storageRPM;
	}
}
