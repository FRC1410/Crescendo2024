package org.frc1410.crescendo2024.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.frc1410.crescendo2024.util.Constants.SWERVE_DRIVE_MAX_ANGULAR_VELOCITY;
import static org.frc1410.crescendo2024.util.Constants.SWERVE_DRIVE_MAX_SPEED;

import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.framework.control.Axis;

public class DriveLooped extends Command {
	private final Drivetrain drivetrain;

	private final Axis xAxis;
	private final Axis yAxis;

	private final Axis rotationAxis;

	private final Axis robotRelativeTrigger;
	
	public DriveLooped(Drivetrain drivetrain, Axis xAxis, Axis yAxis, Axis rotationAxis, Axis robotRelativeTrigger) {
		this.drivetrain = drivetrain;

		this.xAxis = xAxis;
		this.yAxis = yAxis;
		this.rotationAxis = rotationAxis;
		this.robotRelativeTrigger = robotRelativeTrigger;

		this.addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		var xVelocity = SWERVE_DRIVE_MAX_SPEED.times(-this.xAxis.get());
		var yVelocity = SWERVE_DRIVE_MAX_SPEED.times(-this.yAxis.get());
		var angularVelocity = SWERVE_DRIVE_MAX_ANGULAR_VELOCITY.times(-this.rotationAxis.get());

		if (this.robotRelativeTrigger.button().isActive()) {
			// Negate because intake should be forward
			// TODO: does angle need to be negated?
			this.drivetrain.drive(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
		} else {
			this.drivetrain.driveFieldRelative(new ChassisSpeeds(xVelocity.in(MetersPerSecond), yVelocity.in(MetersPerSecond), angularVelocity.in(RadiansPerSecond)));
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
