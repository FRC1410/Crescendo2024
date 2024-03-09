package org.frc1410.crescendo2024.commands;

import org.frc1410.crescendo2024.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotRelative extends Command {
    private final Drivetrain drivetrain;

    public RobotRelative(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.teleopIsFieldRelative = false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.teleopIsFieldRelative = true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
