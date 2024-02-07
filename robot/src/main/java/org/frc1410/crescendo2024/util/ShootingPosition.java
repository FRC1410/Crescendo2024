package org.frc1410.crescendo2024.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootingPosition {

	public final Pose2d pose;
	public final double shooterRPM;

	public ShootingPosition(Pose2d pose, double shooterRPM) {
		this.pose = pose;
		this.shooterRPM = shooterRPM;
	}
}
