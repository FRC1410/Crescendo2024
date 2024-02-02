package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Climb;
import org.frc1410.framework.control.Controller;
import static org.frc1410.chargedup2023.util.Constants.*;

public class ClimbLooped extends Command {

	private final Climb climb;

	public ClimbLooped(Climb climb) {
		this.climb = climb;
		addRequirements(climb);
	}

	@Override
	public void execute() {
		climb.setSpeed(CLIMB_SPEED);
	}

	@Override
	public boolean isFinished() {return false;}

	@Override
	public void end(boolean interrupted) {climb.setSpeed(0);}
}
