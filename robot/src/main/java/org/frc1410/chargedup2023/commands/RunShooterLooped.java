package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Shooter;
import org.frc1410.framework.control.Axis;
import static org.frc1410.chargedup2023.util.Constants.*;


public class RunShooterLooped extends Command {
	private final Shooter shooter;
	private final Axis rightTrigger;

	public RunShooterLooped(Shooter shooter, Axis rightTrigger) {
		this.shooter = shooter;
		this.rightTrigger = rightTrigger;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		double rightTriggerSpeed = rightTrigger.getRaw();
		shooter.setSpeed(rightTriggerSpeed);
	}

	@Override
	public boolean isFinished() {return false;}

	@Override
	public void end(boolean interrupted) {
		shooter.setSpeed(0);
	}
}
