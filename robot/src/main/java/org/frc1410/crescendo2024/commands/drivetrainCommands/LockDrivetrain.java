package org.frc1410.crescendo2024.commands.drivetrainCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Drivetrain;

import static org.frc1410.crescendo2024.util.Constants.SWERVE_DRIVE_MAX_ANGULAR_VELOCITY;
import static org.frc1410.crescendo2024.util.Constants.SWERVE_DRIVE_MAX_SPEED;

public class LockDrivetrain extends Command {

	private final Drivetrain drivetrain;

	public LockDrivetrain(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		drivetrain.lockDrivetrain();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}

