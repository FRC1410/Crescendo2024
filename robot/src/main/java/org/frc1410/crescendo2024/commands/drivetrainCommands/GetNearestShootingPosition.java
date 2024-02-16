package org.frc1410.crescendo2024.commands.drivetrainCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.util.ShootingPosition;

import java.util.function.Consumer;

import static org.frc1410.crescendo2024.util.Constants.SHOOTING_POSITIONS;

public class GetNearestShootingPosition extends Command {
	private final Drivetrain drivetrain;

	private final Consumer<ShootingPosition> consumer;

	public GetNearestShootingPosition(Drivetrain drivetrain, Consumer<ShootingPosition> consumer) {
		this.drivetrain = drivetrain;
		this.consumer = consumer;
	}

	@Override
	public void initialize() {
		Pose2d currentRobotPose = drivetrain.getEstimatedPosition();
		var shootingPoseList = SHOOTING_POSITIONS.stream().map(shootingPositions -> shootingPositions.pose).toList();
		Pose2d nearestPose = currentRobotPose.nearest(shootingPoseList);

		int nearestPoseIndex = shootingPoseList.indexOf(nearestPose);
		double shooterRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).shooterRPM;
		double storageRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).storageRPM;

		this.consumer.accept(SHOOTING_POSITIONS.get(nearestPoseIndex));

		System.out.println(currentRobotPose);
		System.out.println(nearestPose);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
