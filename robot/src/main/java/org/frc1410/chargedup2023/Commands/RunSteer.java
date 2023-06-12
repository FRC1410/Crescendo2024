package org.frc1410.chargedup2023.Commands;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunSteer extends CommandBase {
	private final Drivetrain drivetrain;

	public RunSteer(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

    @Override
	public void initialize() {
		drivetrain.runSteer();
	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.isLocked = false;
	}
}
