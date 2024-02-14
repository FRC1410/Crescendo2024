package org.frc1410.crescendo2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;


public class Outtake extends Command {
	private final Intake intake;
	private final Storage storage;
	private final Shooter shooter;
	private final double intakeOuttakeSpeed;
	private final double storageOuttakeRPM;
	private final double shooterOuttakeRPM;

	public Outtake(Intake intake, Storage storage, Shooter shooter, double intakeOuttakeSpeed, double storageOuttakeRPM, double shooterOuttakeRPM) {
		this.intake = intake;
		this.storage = storage;
		this.shooter = shooter;
		this.intakeOuttakeSpeed = intakeOuttakeSpeed;
		this.storageOuttakeRPM = storageOuttakeRPM;
		this.shooterOuttakeRPM = shooterOuttakeRPM;
		addRequirements(this.intake);
	}

	@Override
	public void initialize() {
		intake.setSpeed(intakeOuttakeSpeed);
		storage.setRPM(storageOuttakeRPM);
		shooter.setRPM(shooterOuttakeRPM);
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
		storage.setRPM(0);
		shooter.setRPM(0);
	}
}
