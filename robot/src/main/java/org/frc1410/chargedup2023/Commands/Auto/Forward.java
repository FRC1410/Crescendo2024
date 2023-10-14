package org.frc1410.chargedup2023.Commands.Auto;

import org.frc1410.chargedup2023.Commands.DriveDistance;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Forward extends SequentialCommandGroup {
    public Forward(Drivetrain drivetrain) {
        this.addCommands(
            new ParallelRaceGroup(
                new DriveDistance(drivetrain, 0),
                new WaitCommand(1)
            )
        );
    }
}
