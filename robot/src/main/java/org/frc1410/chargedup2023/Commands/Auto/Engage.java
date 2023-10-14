package org.frc1410.chargedup2023.Commands.Auto;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Engage extends SequentialCommandGroup {
    public Engage(Drivetrain drivetrain) {
        addCommands(
            new CreepyUntilAngle(drivetrain, false),
            new Balance(drivetrain)
        );
    }
}
