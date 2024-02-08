package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Climb;
import org.frc1410.framework.control.Controller;
import static org.frc1410.chargedup2023.util.Constants.*;

public class ClimbLooped extends Command {

	private final Climb climb;
	private final boolean isReversed;

	public ClimbLooped(Climb climb, boolean isReversed) {
		this.climb = climb;
		this.isReversed = isReversed;
		addRequirements(climb);
	}

	@Override
	public void execute() {
		if(isReversed) {
			climb.setSpeed(-CLIMB_SPEED);
		} else {
			climb.setSpeed(CLIMB_SPEED);
		}
	}

	@Override
	public boolean isFinished() {return false;}

	@Override
	public void end(boolean interrupted) {
		climb.setSpeed(0);
	}
}
