package org.frc1410.crescendo2024.commands.Intake;

import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.LEDs;
import org.frc1410.crescendo2024.subsystems.LEDs.Color;

import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakeStateLEDColor extends Command {
    private final Intake intake;
    private final LEDs leds;

    public SetIntakeStateLEDColor(Intake intake, LEDs leds) {
        this.intake = intake;
        this.leds = leds;

        // Intake lock not needed because we are only reading sensor
        this.addRequirements(leds);
    }

    @Override
    public void execute() {
        this.leds.setColor(
            this.intake.getLimitSwitch() ? Color.PRANCING_PONY_PINK : Color.OCEAN_BREEZE
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
