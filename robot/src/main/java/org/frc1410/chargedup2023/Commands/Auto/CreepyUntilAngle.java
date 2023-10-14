package org.frc1410.chargedup2023.Commands.Auto;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CreepyUntilAngle extends CommandBase {

	private final Drivetrain drivetrain;
	private final boolean reversed;

	public CreepyUntilAngle(Drivetrain drivetrain, boolean reversed) {
		this.drivetrain = drivetrain;
		this.reversed = reversed;

		// addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		if (reversed) {
			drivetrain.drive(4, 0, 0, false);
		} else {
			drivetrain.drive(-4, 0, 0, false);
		}
	}

	@Override
	public void execute() {
	}

	@Override
	public boolean isFinished() {
		return Math.abs(drivetrain.getPitch().getDegrees()) > 13; // Degrees
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
	}
}
