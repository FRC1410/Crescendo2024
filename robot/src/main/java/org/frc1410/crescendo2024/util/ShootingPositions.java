package org.frc1410.crescendo2024.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootingPositions {

	public final Pose2d pose;
	private final double shooterRPM;

	public ShootingPositions(Pose2d pose, double shooterRPM) {
		this.pose = pose;
		this.shooterRPM = shooterRPM;
	}
}
