package org.frc1410.crescendo2024.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.Subsystems.Drivetrain;
import org.frc1410.framework.control.Axis;

public class DriveLooped extends Command {
	private final Drivetrain drivetrain;

	private final Axis xAxis;
	private final Axis yAxis;

	private final Axis rotationAxis;
	
	public DriveLooped(Drivetrain drivetrain, Axis xAxis, Axis yAxis, Axis rotationAxis) {
		this.drivetrain = drivetrain;

		this.xAxis = xAxis;
		this.yAxis = yAxis;
		this.rotationAxis = rotationAxis;

		this.addRequirements(drivetrain);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		var xVelocity = xAxis.get() * 4;
		var yVelocity = yAxis.get() * 4;
		var rotation = rotationAxis.get() * 4;
		drivetrain.driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, rotation));
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
