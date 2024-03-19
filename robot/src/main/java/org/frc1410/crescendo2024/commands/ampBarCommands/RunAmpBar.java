// package org.frc1410.crescendo2024.commands.ampBarCommands;

// import org.frc1410.crescendo2024.subsystems.AmpBar;

// import edu.wpi.first.wpilibj2.command.Command;
// import org.frc1410.crescendo2024.subsystems.LEDs;

// import static org.frc1410.crescendo2024.util.Constants.*;

// public class RunAmpBar extends Command {

// 	private final AmpBar ampBar;
// 	private final int direction;

// 	private boolean isExtended;

// 	public RunAmpBar(AmpBar ampBar, int direction) {
// 		this.ampBar = ampBar;
// 		this.direction = direction;
// 		addRequirements(this.ampBar);
// 	}

// 	@Override
// 	public void initialize() {
// 		ampBar.setSpeed(AMP_BAR_SPEED * direction);
// 	}

// 	@Override
// 	public void execute() {

// 	}

// 	@Override
// 	public boolean isFinished() {
// 		return false;
// 	}

// 	@Override
// 	public void end(boolean interrupted) {
// 		ampBar.setSpeed(0);
// 	}
// }