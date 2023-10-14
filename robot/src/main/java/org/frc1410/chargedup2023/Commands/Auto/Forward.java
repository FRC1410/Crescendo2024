package org.frc1410.chargedup2023.Commands.Auto;

import org.frc1410.chargedup2023.Commands.DriveDistance;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Forward extends SequentialCommandGroup {
    public Forward(Drivetrain drivetrain) {
        this.addCommands(
            new DriveDistance(drivetrain, 1)
        );
    }
}
