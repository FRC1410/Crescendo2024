package org.frc1410.crescendo2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2023.subsystems.Climb;
import org.frc1410.framework.control.Axis;
import static org.frc1410.crescendo2023.util.Constants.*;


public class ClimbLooped extends Command {
	private final Climb climb;
	private final Axis leftY;

	public ClimbLooped(Climb climb, Axis leftY) {
		this.climb = climb;
		this.leftY = leftY;

		addRequirements(this.climb);
	}


	@Override
	public void initialize() {

	}


	@Override
	public void execute() {
		if(leftY.get() > .2) {
			climb.setSpeeds(CLIMB_SPEEDS);
		}
		else if(leftY.get() < .2) {
			climb.setSpeeds(-CLIMB_SPEEDS);
		}

	}

	@Override
	public boolean isFinished() {return false;}

	@Override
	public void end(boolean interrupted) {
		climb.setSpeeds(0);
	}
}
