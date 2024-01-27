package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Shooter;
import static  org.frc1410.chargedup2023.util.Constants.*;


public class ShooterManual extends Command {
	private final Shooter shooter;

	public ShooterManual(Shooter shooter) {
		this.shooter = shooter;
		addRequirements(this.shooter);
	}

	@Override
	public void execute() {shooter.setRPM(shooter.shooterSpeed);}

	@Override
	public boolean isFinished() {return false;}

	@Override
	public void end(boolean interrupted) {shooter.setRPM(0);}
}
