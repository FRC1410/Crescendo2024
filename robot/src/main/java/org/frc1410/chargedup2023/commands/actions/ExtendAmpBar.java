package org.frc1410.chargedup2023.commands.actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.frc1410.chargedup2023.subsystems.AmpBar;
import static org.frc1410.chargedup2023.util.Tuning.*;




public class ExtendAmpBar extends Command {

	private final AmpBar ampBar;

	private final int pos;

	private PIDController pid;

	public ExtendAmpBar(AmpBar ampBar, int pos) {
		this.ampBar = ampBar;
		this.pos = pos;
		addRequirements(ampBar);
	}

	@Override
	public void initialize() {
		ampBar.setDesiredPosition(pos);
		pid = new PIDController(AMP_KP, AMP_KI, AMP_KD);
		pid.setTolerance(AMP_TOLERANCE);
	}

	@Override
	public boolean isFinished() {
		return pid.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
	}
}
