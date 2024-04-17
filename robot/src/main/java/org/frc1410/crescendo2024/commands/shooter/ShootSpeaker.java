package org.frc1410.crescendo2024.commands.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.frc1410.crescendo2024.util.Constants.*;

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
            new WaitCommand(SHOOTING_TIME.in(Seconds)),
            new ParallelCommandGroup(
                new RunStorage(storage, SPEAKER_STORAGE_VELOCITY),
                new RunUnderBumperIntake(intake, INTAKE_SPEED)
            )
        );
    }
}
