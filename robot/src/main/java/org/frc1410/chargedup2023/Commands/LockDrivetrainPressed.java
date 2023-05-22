package org.frc1410.chargedup2023.Commands;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockDrivetrainPressed extends CommandBase {
    private final Drivetrain drivetrain;

	public LockDrivetrainPressed(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

    @Override
	public void initialize() {
		drivetrain.isLocked = true;
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
        
	}
}