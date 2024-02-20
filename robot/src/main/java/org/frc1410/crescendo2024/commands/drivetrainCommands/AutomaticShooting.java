package org.frc1410.crescendo2024.commands.drivetrainCommands;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.sun.jdi.ShortType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.frc1410.crescendo2024.commands.shooterCommands.RunShooterLooped;
import org.frc1410.crescendo2024.subsystems.*;
import org.frc1410.crescendo2024.util.ShootingPosition;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;

import static org.frc1410.crescendo2024.util.Constants.*;

public class AutomaticShooting extends Command {

	private final Drivetrain drivetrain;

	private final Shooter shooter;
	private final Storage storage;
	private final Intake intake;
	private double storageRPM;
	private final Timer timer = new Timer();
	private boolean storageIsRunning = false;
	private PathfindThenFollowPathHolonomic pathfindHolonomic;
	private PathPlannerPath pathPlannerPath;

	private Pose2d nearestPose;


	public AutomaticShooting(Drivetrain drivetrain, Storage storage, Intake intake, Shooter shooter) {
		this.drivetrain = drivetrain;
		this.storage = storage;
		this.shooter = shooter;
		this.intake = intake;
		addRequirements(drivetrain);

//		PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
	}

	public Optional<Rotation2d> getRotationTargetOverride() {
		var r = this.drivetrain.getEstimatedPosition();
		var a = new Translation2d(0.0381, 5.55);

		if(this.nearestPose != null && this.nearestPose.getTranslation().getDistance(this.drivetrain.getEstimatedPosition().getTranslation()) > 0.5) {
			return Optional.empty();
		}

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

		this.nearestPose = nearestPose;

		int nearestPoseIndex = shootingPoseList.indexOf(nearestPose);
		double shooterRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).shooterRPM;
		storageRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).storageRPM;
		String pathName = SHOOTING_POSITIONS.get(nearestPoseIndex).pathName;

		pathPlannerPath = PathPlannerPath.fromPathFile(pathName);

		System.out.println("current " + currentRobotPose);
		System.out.println("nearest " + nearestPose);

		//this.command = new AutomaticShooting(this.drivetrain, this.shooter, this.storage, SHOOTING_POSITIONS.get(nearestPoseIndex));

		pathfindHolonomic = new PathfindThenFollowPathHolonomic(
			pathPlannerPath,
			PATH_FIND_CONSTRAINTS,
			drivetrain::getEstimatedPosition,
			drivetrain::getChassisSpeeds,
			drivetrain::drive,
			PATH_FIND_FOLLOWER_CONFIG,
			0,
			() -> {
				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
			},
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
				intake.setSpeed(0.75);
				storageIsRunning = true;
//				timer.start();
			}
		}
	}

	@Override
	public boolean isFinished() {
//		return timer.hasElapsed(1);
//		if (this.pathfindHolonomic != null) {
//			return this.pathfindHolonomic.isFinished();
//		}
//		return true;
		return false;
	}

	@Override
	public void end(boolean interrupted) {
//		if(pathfindHolonomic != null) {
//			pathfindHolonomic.end(interrupted);
//		}
//		storage.setRPM(0);
//		shooter.setRPM(0);
//		intake.setSpeed(0);
//		System.out.println("end " + drivetrain.getEstimatedPosition());
	}
}
