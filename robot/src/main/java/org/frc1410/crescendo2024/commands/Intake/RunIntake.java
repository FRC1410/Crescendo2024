package org.frc1410.crescendo2024.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Intake;

public class RunIntake extends Command {
	private final Intake intake;

	private final double speed;

	public RunIntake(Intake intake, double speed) {
		this.intake = intake;
		this.speed = speed;

		this.addRequirements(this.intake);
	}

	@Override
	public void initialize() {
		this.intake.setSpeed(this.speed);
		this.intake.setExtended(true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.setSpeed(0);
		this.intake.setExtended(false);
	}
}
