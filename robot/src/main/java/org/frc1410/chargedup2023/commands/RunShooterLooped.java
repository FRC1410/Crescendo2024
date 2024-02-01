package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Shooter;
import org.frc1410.framework.control.Axis;
import org.frc1410.framework.control.Controller;


public class RunShooterLooped extends Command {
	private final Shooter shooter;
	private final double poseRPM;

	public RunShooterLooped(Shooter shooter, double poseRPM) {
		this.shooter = shooter;
		addRequirements(shooter);
		this.poseRPM = poseRPM;
	}

	@Override
	public void execute() {
		shooter.setRPM(poseRPM);
	}

	@Override
	public boolean isFinished() {return false;}

	@Override
	public void end(boolean interrupted) {shooter.setRPM(0);}
}