package org.frc1410.crescendo2024.commands.Intake;

import org.frc1410.crescendo2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class FlipIntake extends Command {
    private final Intake intake;

    public FlipIntake(Intake intake) {
        this.intake = intake;

        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.intake.setExtended(!this.intake.isExtended());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
