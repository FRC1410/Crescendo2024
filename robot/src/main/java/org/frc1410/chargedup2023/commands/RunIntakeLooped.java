package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Intake;
import org.frc1410.framework.control.Axis;
import static org.frc1410.chargedup2023.util.Constants.*;


public class RunIntakeLooped extends Command {
	private final Intake intake;
	private final Axis leftTrigger;
	private final Axis rightTrigger;

	public RunIntakeLooped(Intake intake, Axis leftTrigger, Axis rightTrigger) {
		this.intake = intake;
		this.leftTrigger = leftTrigger;
		this.rightTrigger = rightTrigger;

		addRequirements(intake);
	}


	@Override
	public void initialize() {

	}


	@Override
	public void execute() {
		if(leftTrigger.get() > 0.3) {
			intake.setSpeed(INTAKE_SPEED);
		}
		if(rightTrigger.get() > .03) {
			intake.setSpeed(-INTAKE_SPEED);
		}
	}


	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}


	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0);

	}
}
