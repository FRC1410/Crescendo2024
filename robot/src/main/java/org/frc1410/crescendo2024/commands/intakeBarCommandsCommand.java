package org.frc1410.crescendo2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.IntakeBar;
import static org.frc1410.crescendo2024.util.Constants.*;

public class intakeBarCommandsCommand extends Command {
	private final IntakeBar intakeBar;
	private final int direction;

	public intakeBarCommandsCommand(IntakeBar intakeBar, int direction) {
		this.direction =direction;
		this.intakeBar = intakeBar;
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.intakeBar);
	}

	@Override
	public void initialize() {
		intakeBar.setSpeed(INTAKE_BAR_SPEED * direction);
	}

	@Override
	public void execute() {
	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}


	@Override
	public void end(boolean interrupted) {
		 intakeBar.setSpeed(0.0);
	}
}
