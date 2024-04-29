package org.frc1410.crescendo2024.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

public class AdjustShooterRPM extends Command {
	private final Shooter shooter;

	private final Measure<Velocity<Angle>> adjustment;

	private final boolean amp;

	public AdjustShooterRPM(Shooter shooter, Measure<Velocity<Angle>> adjustment, boolean amp) {
		this.shooter = shooter;
		this.adjustment = adjustment;
		this.amp = amp;

		this.addRequirements(shooter);
	}

	@Override
	public void initialize() {
		if (this.amp) {
			this.shooter.setAmpVelocityAdjustment(this.shooter.getAmpVelocityAdjustment().plus(this.adjustment));
		} else {
			this.shooter.setSpeakerVelocityAdjustment(this.shooter.getSpeakerVelocityAdjustment().plus(this.adjustment));
		}
		
		System.out.println("RPM ADJUSTMENT: " + shooter.getAmpVelocityAdjustment().in(RPM));
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
