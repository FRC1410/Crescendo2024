package org.frc1410.crescendo2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class ShootingPosition {
	public final Pose2d pose;
	public final Measure<Velocity<Angle>> shooterVelocity;
	public final Measure<Velocity<Angle>> storageVelocity;

	public ShootingPosition(Pose2d pose, Measure<Velocity<Angle>> shooterVelocity, Measure<Velocity<Angle>> storageVelocity) {
		this.pose = pose;
		this.shooterVelocity = shooterVelocity;
		this.storageVelocity = storageVelocity;
	}
}
