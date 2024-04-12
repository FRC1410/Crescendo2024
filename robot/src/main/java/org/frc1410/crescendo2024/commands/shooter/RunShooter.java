package org.frc1410.crescendo2024.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

public class RunShooter extends Command {
	private final Shooter shooter;

	private final Measure<Velocity<Angle>> velocity;

	private final boolean useAdjustment;

	public RunShooter(Shooter shooter, Measure<Velocity<Angle>> velocity, boolean useAdjustment) {
		this.shooter = shooter;
		this.velocity = velocity;
		this.useAdjustment = useAdjustment;

		this.addRequirements(shooter);
	}

	@Override
	public void initialize() {
		var velocity = this.useAdjustment
			? this.velocity.plus(shooter.velocityAdjustment)
			: this.velocity;
		
		this.shooter.setVelocity(velocity);
	}

	@Override
	public void execute() {
		var velocity = this.useAdjustment
			? this.velocity.plus(shooter.velocityAdjustment)
			: this.velocity;
		
		this.shooter.setVelocity(velocity);
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