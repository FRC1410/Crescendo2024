package org.frc1410.crescendo2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Intake;


public class RunIntake extends Command {
	private final Intake intake;

	private final double speed;

	public RunIntake(Intake intake, double speed) {
		this.intake = intake;
		this.speed = speed;
		addRequirements(this.intake);
	}

	@Override
	public void initialize() {
		intake.setSpeed(speed);
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
		intake.setSpeed(0);
	}
}
