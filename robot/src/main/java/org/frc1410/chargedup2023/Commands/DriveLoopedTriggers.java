package org.frc1410.chargedup2023.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;
import org.frc1410.framework.control.Axis;

public class DriveLoopedTriggers extends CommandBase {
    private final Drivetrain drivetrain;

	private final Axis rightXAxis;
	private final Axis rightYAxis;

	private final Axis leftTrigger;
    private final Axis rightTrigger;

	private boolean previousTickHadInput = false;
	
	public DriveLoopedTriggers(Drivetrain drivetrain, Axis rightXAxis, Axis rightYAxis, Axis leftTrigger, Axis rightTrigger) {
		this.drivetrain = drivetrain;
		this.rightXAxis = rightXAxis;
		this.rightYAxis = rightYAxis;
		this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		var xVelocity = rightYAxis.get();
		var yVelocity = rightXAxis.get();
		var rotation = rightTrigger.get() - leftTrigger.get();

		var hasInput = xVelocity != 0 || yVelocity != 0 || rotation != 0;

		if (!previousTickHadInput && hasInput) {
			drivetrain.isLocked = false;
		}

		drivetrain.drive(rightYAxis.get(), rightXAxis.get(), rotation, true);

		previousTickHadInput = hasInput;
	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
