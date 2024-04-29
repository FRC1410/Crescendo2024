package org.frc1410.crescendo2024.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.framework.control.Controller;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

public class RunShooter extends Command {
	private final Shooter shooter;

	private final Measure<Velocity<Angle>> velocity;

	private final boolean speaker;

	private final Controller controller;

	private boolean didRumble = false;

	public RunShooter(Shooter shooter, Measure<Velocity<Angle>> velocity, boolean speaker, Controller controller) {
		this.shooter = shooter;
		this.velocity = velocity;
		this.speaker = speaker;
		this.controller = controller;

		this.addRequirements(shooter);
	}

	@Override
	public void initialize() {
		var velocity = this.speaker
			? this.velocity.plus(shooter.getSpeakerVelocityAdjustment())
			: this.velocity.plus(shooter.getAmpVelocityAdjustment());
		
		this.shooter.setVelocity(velocity);
	}

	@Override
	public void execute() {
		var velocity = this.speaker
			? this.velocity.plus(shooter.getSpeakerVelocityAdjustment())
			: this.velocity.plus(shooter.getAmpVelocityAdjustment());
		
		this.shooter.setVelocity(velocity);

		if (Math.abs(this.shooter.getVelocity().in(RPM) - this.velocity.in(RPM)) < 100 && !this.didRumble) {
			if (this.controller != null) {
				this.controller.rumble(500);
			}
			this.didRumble = true;
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.shooter.setVelocity(RPM.zero());
	}
}