package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Shooter;
import org.frc1410.framework.control.Axis;
import org.frc1410.framework.control.Controller;


public class RunShooterLooped extends Command {
	private final Shooter shooter;
	private final Controller operatorController;



	public RunShooterLooped(Shooter shooter, Controller OperatorController) {
		this.shooter = shooter;
		this.operatorController = OperatorController;
		addRequirements(shooter);
	}
	@Override
	public void execute() {
		if(operatorController.RIGHT_TRIGGER.get() > 1) {
			shooter.setRPM(1);
		}
	}

	@Override
	public boolean isFinished() {return false;}
	@Override
	public void end(boolean interrupted) {shooter.setRPM(0);}
}