package org.frc1410.crescendo2024.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

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
	
	public DriveLooped(Drivetrain drivetrain, Axis xAxis, Axis yAxis, Axis rotationAxis, Axis trigger) {
		this.drivetrain = drivetrain;

		this.xAxis = xAxis;
		this.yAxis = yAxis;
		this.rotationAxis = rotationAxis;
		this.robotRelativeTrigger = trigger;

		this.addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		// TODO: these shouldn't be flipped here
		var xVelocity = SWERVE_DRIVE_MAX_SPEED.times(this.yAxis.get()).negate();
		var yVelocity = SWERVE_DRIVE_MAX_SPEED.times(this.xAxis.get()).negate();
		var angularVelocity = SWERVE_DRIVE_MAX_ANGULAR_VELOCITY.times(this.rotationAxis.get()).negate();

		if (this.robotRelativeTrigger.button().isActive()) {
			this.drivetrain.drive(new ChassisSpeeds(xVelocity.negate(), yVelocity.negate(), angularVelocity));
		} else {
			this.drivetrain.driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
