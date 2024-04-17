package org.frc1410.crescendo2024.commands.drivetrain;

import org.frc1410.crescendo2024.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class LockDrivetrain extends Command {
    private final Drivetrain drivetrain;

    public LockDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.lockWheels();
    }
}
