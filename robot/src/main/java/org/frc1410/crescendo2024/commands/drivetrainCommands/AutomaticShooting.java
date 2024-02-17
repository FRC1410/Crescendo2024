package org.frc1410.crescendo2024.commands.drivetrainCommands;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.sun.jdi.ShortType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.frc1410.crescendo2024.commands.shooterCommands.RunShooterLooped;
import org.frc1410.crescendo2024.subsystems.Camera;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;
import org.frc1410.crescendo2024.util.ShootingPosition;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;

import static org.frc1410.crescendo2024.util.Constants.*;

public class AutomaticShooting extends Command {

	private final Drivetrain drivetrain;

	private final Shooter shooter;
	private final Storage storage;
	private double storageRPM;
	private final Timer timer = new Timer();
	private boolean storageIsRunning = false;
	private PathfindHolonomic pathfindHolonomic;


	public AutomaticShooting(Drivetrain drivetrain, Storage storage, Shooter shooter) {
		this.drivetrain = drivetrain;
		this.storage = storage;
		this.shooter = shooter;
		addRequirements(drivetrain);

		PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
	}

	public Optional<Rotation2d> getRotationTargetOverride() {
		var r = this.drivetrain.getEstimatedPosition();
		var a = new Translation2d(0.0381, 5.55);
		return Optional.of(Rotation2d.fromRadians(
			MathUtil.angleModulus(Math.PI + Math.atan(
				(r.getY() - a.getY())/
					(r.getX() - a.getX())
			))
		));
	}

	@Override
	public void initialize() {
		timer.stop();
		timer.reset();

		storageIsRunning = false;

		Pose2d currentRobotPose = drivetrain.getEstimatedPosition();
		var shootingPoseList = SHOOTING_POSITIONS.stream().map(shootingPositions -> shootingPositions.pose).toList();
		Pose2d nearestPose = currentRobotPose.nearest(shootingPoseList);

		int nearestPoseIndex = shootingPoseList.indexOf(nearestPose);
		double shooterRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).shooterRPM;
		storageRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).storageRPM;

		System.out.println("current " + currentRobotPose);
		System.out.println("nearest " + nearestPose);

		//this.command = new AutomaticShooting(this.drivetrain, this.shooter, this.storage, SHOOTING_POSITIONS.get(nearestPoseIndex));

		pathfindHolonomic = new PathfindHolonomic(
			nearestPose,
			PATH_FIND_CONSTRAINTS,
			0.0,
			drivetrain::getEstimatedPosition,
			drivetrain::getChassisSpeeds,
			drivetrain::drive,
			PATH_FIND_FOLLOWER_CONFIG,
			0,
			drivetrain
		);

		pathfindHolonomic.initialize();
		shooter.setRPM(shooterRPM);
	}

	@Override
	public void execute() {
		if(pathfindHolonomic != null) {
			if(!pathfindHolonomic.isFinished()) {
				pathfindHolonomic.execute();
			} else if(!storageIsRunning) {
				pathfindHolonomic.end(false);
				storage.setRPM(storageRPM);
				storageIsRunning = true;
				timer.start();
			}
		}
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(1);
//		if (this.pathfindHolonomic != null) {
//			return this.pathfindHolonomic.isFinished();
//		}
//		return true;
	}

	@Override
	public void end(boolean interrupted) {
		if(pathfindHolonomic != null) {
			pathfindHolonomic.end(interrupted);
		}
		storage.setRPM(0);
		shooter.setRPM(0);
		System.out.println("end " + drivetrain.getEstimatedPosition());
	}
}
