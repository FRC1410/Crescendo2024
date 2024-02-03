package org.frc1410.crescendo2024.Commands;

import com.pathplanner.lib.commands.PathfindHolonomic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.Subsystems.Camera;
import org.frc1410.crescendo2024.Subsystems.Drivetrain;

import static org.frc1410.crescendo2024.util.Constants.*;

public class DriveToShootingPose extends Command {

	private final Drivetrain drivetrain;
	private final Camera camera;

	private PathfindHolonomic pathfindHolonomic;

	public DriveToShootingPose(Drivetrain drivetrain, Camera camera) {
		this.drivetrain = drivetrain;
		this.camera = camera;
		addRequirements(drivetrain, camera);
	}

	@Override
	public void initialize() {

		Pose2d currentRobotPose = drivetrain.getEstimatedPosition();
		Pose2d nearestPose = currentRobotPose.nearest(SHOOTING_POSITIONS.stream().map(shootingPositions -> shootingPositions.pose).toList());

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
