package org.frc1410.crescendo2024.commands.Intake;

import org.frc1410.crescendo2024.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class Fuck extends Command {
    private final Intake intake;

    public Fuck(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setSpeed(0.05);

    }
}
