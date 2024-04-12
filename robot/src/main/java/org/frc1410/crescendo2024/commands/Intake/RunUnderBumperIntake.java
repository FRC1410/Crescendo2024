package org.frc1410.crescendo2024.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Intake;

public class RunUnderBumperIntake extends Command {
	private final Intake intake;

	private final double speed;

	public RunUnderBumperIntake(Intake intake, double speed) {
		this.intake = intake;
		this.speed = speed;

		this.addRequirements(this.intake);
	}

	@Override
	public void initialize() {
		this.intake.setUnderBumperSpeed(this.speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.setUnderBumperSpeed(0);
	}
}
