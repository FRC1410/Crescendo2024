package org.frc1410.crescendo2024.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitForShooterVelocity extends Command {
    private final Shooter shooter;
    private final Measure<Velocity<Angle>> target;
    private final Timer timeoutTimer = new Timer();

    public WaitForShooterVelocity(Shooter shooter, Measure<Velocity<Angle>> target) {
        this.shooter = shooter;
        this.target = target;
    }

    @Override
    public void initialize() {
        this.timeoutTimer.start();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.shooter.getVelocity().in(RPM) - this.target.in(RPM)) < 70 || timeoutTimer.hasElapsed(1);
    }
}
