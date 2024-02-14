package org.frc1410.crescendo2024.commands.drivetrainCommands;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.sun.jdi.ShortType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.frc1410.crescendo2024.commands.shooterCommands.RunShooterLooped;
import org.frc1410.crescendo2024.subsystems.Camera;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.util.ShootingPosition;

import java.util.List;

import static org.frc1410.crescendo2024.util.Constants.*;

public class DriveToShootingPose extends Command {

	private final Drivetrain drivetrain;
	private final Shooter shooter;

	private PathfindHolonomic pathfindHolonomic;

	public DriveToShootingPose(Drivetrain drivetrain, Shooter shooter) {
		this.drivetrain = drivetrain;
		this.shooter = shooter;
		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {

		Pose2d currentRobotPose = drivetrain.getEstimatedPosition();
		Pose2d nearestPose = currentRobotPose.nearest(SHOOTING_POSITIONS.stream().map(shootingPositions -> shootingPositions.pose).toList());

		int nearestPoseIndex = SHOOTING_POSITIONS.indexOf(nearestPose);
		ShootingPosition shootingPose = SHOOTING_POSITIONS.get(nearestPoseIndex);
		double shooterRPM = shootingPose.shooterRPM;

		pathfindHolonomic = new PathfindHolonomic(
			nearestPose,
			PATH_FIND_CONSTRAINTS,
			drivetrain::getEstimatedPosition,
			drivetrain::getChassisSpeeds,
			drivetrain::drive,
			PATH_FIND_FOLLOWER_CONFIG,
			0.0,
			drivetrain
		);

		pathfindHolonomic.initialize();
	}

	@Override
	public void execute() {
		if(pathfindHolonomic != null) {
			pathfindHolonomic.execute();
		}
	}

	@Override
	public boolean isFinished() {
		if(pathfindHolonomic != null) {
			pathfindHolonomic.isFinished();
		}
		return false;
    }

	@Override
	public void end(boolean interrupted) {
		if(pathfindHolonomic != null) {
			pathfindHolonomic.end(interrupted);
		}
	}
}
