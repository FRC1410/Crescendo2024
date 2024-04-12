package org.frc1410.crescendo2024.commands.shooter;

import static org.frc1410.crescendo2024.util.Constants.MANUAL_INTAKE_SPEED;
import static org.frc1410.crescendo2024.util.Constants.MANUAL_STORAGE_RPM;
import static org.frc1410.crescendo2024.util.Constants.SHOOTING_TIME;

import org.frc1410.crescendo2024.commands.RunStorage;
import org.frc1410.crescendo2024.commands.intake.RunUnderBumperIntake;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootSpeaker extends ParallelRaceGroup {
    public ShootSpeaker(Storage storage, Intake intake) {
        this.addCommands(
            new WaitCommand(SHOOTING_TIME),
            new ParallelCommandGroup(
                new RunStorage(storage, MANUAL_STORAGE_RPM),
                new RunUnderBumperIntake(intake, MANUAL_INTAKE_SPEED)
            )
        );
    }
}
