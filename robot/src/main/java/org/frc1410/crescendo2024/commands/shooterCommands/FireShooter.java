package org.frc1410.crescendo2024.commands.shooterCommands;

import static org.frc1410.crescendo2024.util.Constants.MANUAL_INTAKE_SPEED;
import static org.frc1410.crescendo2024.util.Constants.MANUAL_STORAGE_RPM;

import org.frc1410.crescendo2024.commands.RunStorage;
import org.frc1410.crescendo2024.commands.Intake.RunIntake;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FireShooter extends ParallelCommandGroup {
    public FireShooter(Storage storage, Intake intake) {
        this.addCommands(
            new RunStorage(storage, MANUAL_STORAGE_RPM),
            new RunIntake(intake, MANUAL_INTAKE_SPEED)
        );
    }
}
