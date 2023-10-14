package org.frc1410.chargedup2023.Commands;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
    private final Drivetrain drivetrain;

    private final double distanceMeters;

    public DriveDistance(Drivetrain drivetrain, double distanceMeters) {
        this.drivetrain = drivetrain;
        this.distanceMeters = distanceMeters;
    }

    @Override
    public void execute() {
        this.drivetrain.drive(0, 4, 0, true);
    }

    @Override
    public boolean isFinished() {
        return this.drivetrain.getPoseMeters().getY() >= this.distanceMeters;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
