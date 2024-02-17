package org.frc1410.crescendo2024.commands.drivetrainCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc1410.crescendo2024.commands.RunStorage;
import org.frc1410.crescendo2024.commands.shooterCommands.RunShooterLooped;
import org.frc1410.crescendo2024.commands.shooterCommands.Shoot;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;
import org.frc1410.crescendo2024.util.ShootingPosition;

import static org.frc1410.crescendo2024.util.Constants.SHOOTING_POSITIONS;

public class AutomaticShooting extends SequentialCommandGroup {

	public AutomaticShooting(Drivetrain drivetrain, Shooter shooter, Storage storage) {

		Pose2d currentRobotPose = drivetrain.getEstimatedPosition();
		var shootingPoseList = SHOOTING_POSITIONS.stream().map(shootingPositions -> shootingPositions.pose).toList();
		Pose2d nearestPose = currentRobotPose.nearest(shootingPoseList);

		int nearestPoseIndex = shootingPoseList.indexOf(nearestPose);
		double shooterRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).shooterRPM;
		double storageRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).storageRPM;

		System.out.println(currentRobotPose);
		System.out.println(nearestPose);

		addCommands(
			new ParallelRaceGroup(
				new RunShooterLooped(shooter, shooterRPM),

				new SequentialCommandGroup(
					new DriveToShootingPose(drivetrain, nearestPose),

					new ParallelRaceGroup(
						new WaitCommand(0.5),
						new RunStorage(storage, storageRPM)
					)
				)
			)
		);
	}
}
