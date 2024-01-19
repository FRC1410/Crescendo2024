package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Shooter;
import org.frc1410.framework.control.Axis;
import static org.frc1410.chargedup2023.util.Constants.*;
import org.frc1410.framework.control.Controller;


public class RunShooterLooped extends Command {
	private final Shooter shooter;
	private final Controller operatorController;
	private final Axis rightTrigger;

	public RunShooterLooped(Shooter shooter, Controller OperatorController) {
		this.shooter = shooter;
		this.operatorController = OperatorController;
		addRequirements(shooter);
		rightTrigger = operatorController.RIGHT_TRIGGER;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if (rightTrigger.getRaw() > 0.2) {
			shooter.setSpeed(rightTrigger.getRaw());
		} else if (operatorController.Y.isActive()) {
			shooter.setSpeed(0.9);
		} else if (operatorController.B.isActive()) {
			shooter.setSpeed(0.8);
		} else if (operatorController.A.isActive()) {
			shooter.setSpeed(0.7);
		} else if (operatorController.X.isActive()) {
			shooter.setSpeed(0.6);
		}
		/*
		double rightTriggerSpeed = rightTrigger.getRaw();
		for testing purposes
		if (rightTriggerSpeed >= 0.0) {
			shooter.setSpeed(0.9);
		}

		shooter.setSpeed(rightTriggerSpeed);
		*/
	}

	@Override
	public boolean isFinished() {return false;}

	@Override
	public void end(boolean interrupted) {
		shooter.setSpeed(0);
	}
}
