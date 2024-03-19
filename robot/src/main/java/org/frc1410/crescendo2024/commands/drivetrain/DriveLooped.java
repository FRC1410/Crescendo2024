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

	private final Axis trigger;
	
	public DriveLooped(Drivetrain drivetrain, Axis xAxis, Axis yAxis, Axis rotationAxis, Axis trigger) {
		this.drivetrain = drivetrain;

		this.xAxis = xAxis;
		this.yAxis = yAxis;
		this.rotationAxis = rotationAxis;
		this.trigger = trigger;

		this.addRequirements(drivetrain);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		var xVelocity = -yAxis.get() * SWERVE_DRIVE_MAX_SPEED;
		var yVelocity = -xAxis.get() * SWERVE_DRIVE_MAX_SPEED;
		var angularVelocity = -rotationAxis.get() * SWERVE_DRIVE_MAX_ANGULAR_VELOCITY;

		if (trigger.button().isActive()) {
			drivetrain.drive(new ChassisSpeeds(-xVelocity, -yVelocity, angularVelocity));
		} else {
			drivetrain.driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
