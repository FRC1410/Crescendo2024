package org.frc1410.chargedup2023.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;
import org.frc1410.framework.control.Axis;


public class DriveLooped extends CommandBase {
	private final Drivetrain drivetrain;

	private final Axis rightXAxis;
	private final Axis rightYAxis;

	private final Axis leftXAxis;

	private boolean previousTickHadInput = false;
	
	public DriveLooped(Drivetrain drivetrain, Axis rightXAxis, Axis rightYAxis, Axis leftXAxis) {
		this.drivetrain = drivetrain;
		this.rightXAxis = rightXAxis;
		this.rightYAxis = rightYAxis;
		this.leftXAxis = leftXAxis;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		var xVelocity = rightYAxis.get();
		var yVelocity = rightXAxis.get();
		var rotation = leftXAxis.get();

		var hasInput = xVelocity != 0 || yVelocity != 0 || rotation != 0;

		if (!previousTickHadInput && hasInput) {
			drivetrain.isLocked = false;
		}

		drivetrain.drive(rightYAxis.get(), rightXAxis.get(), leftXAxis.get(), true);

		previousTickHadInput = hasInput;
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
