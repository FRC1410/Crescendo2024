package org.frc1410.crescendo2024.commands.drivetrainCommands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
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
import org.frc1410.crescendo2024.subsystems.LEDs.Colors;
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
	private final LEDs leds;

	private ShootingPosition shootingPosition;
	private boolean storageIsRunning = false;

	private final Timer timer = new Timer();

	private FollowPathHolonomic followPathCommand = null;

	public AutomaticShooting(Drivetrain drivetrain, Storage storage, Intake intake, Shooter shooter, LEDs leds) {
		this.drivetrain = drivetrain;
		this.storage = storage;
		this.shooter = shooter;
		this.intake = intake;
		this.leds = leds;
		addRequirements(drivetrain, storage, shooter, intake, leds);

//		PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
	}

	// public Optional<Rotation2d> getRotationTargetOverride() {
	// 	var r = this.drivetrain.getEstimatedPosition();
	// 	var a = new Translation2d(0.0381, 5.55);

	// 	if(this.nearestPose != null && this.nearestPose.getTranslation().getDistance(this.drivetrain.getEstimatedPosition().getTranslation()) > 0.5) {
	// 		return Optional.empty();
	// 	}

	// 	return Optional.of(Rotation2d.fromRadians(
	// 		MathUtil.angleModulus(Math.PI + Math.atan(
	// 			(r.getY() - a.getY())/
	// 				(r.getX() - a.getX())
	// 		))
	// 	));
	// }

	@Override
	public void initialize() {
		var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
		var shootingPositions = (alliance == DriverStation.Alliance.Blue) ? SHOOTING_POSITIONS_BLUE : SHOOTING_POSITIONS_RED;
		
		timer.stop();
		timer.reset();

		storageIsRunning = false;

		Pose2d currentRobotPose = drivetrain.getEstimatedPosition();
		// var shootingPoseList = shootingPositions.stream().map(x -> x.pose).toList();
		// Pose2d nearestPose = currentRobotPose.nearest(shootingPoseList);
		
		ShootingPosition nearestPosition = null;
		double smallestDistance = Double.MAX_VALUE;
		
		for (ShootingPosition position : shootingPositions) {
			var distance = position.pose.getTranslation().getDistance(currentRobotPose.getTranslation());
			if (distance < smallestDistance) {
				smallestDistance = distance;
				nearestPosition = position;
			}
		}

		// this.nearestPose = nearestPose;

		// int nearestPoseIndex = shootingPoseList.indexOf(nearestPose);
		// double shooterRPM = shootingPositions.get(nearestPoseIndex).shooterRPM;
		// var storageRPM = shootingPositions.get(nearestPoseIndex).storageRPM;

		this.shootingPosition = nearestPosition;

		// System.out.println("current " + currentRobotPose);
		// System.out.println("nearest " + nearestPosition.pose);

		//this.command = new AutomaticShooting(this.drivetrain, this.shooter, this.storage, SHOOTING_POSITIONS.get(nearestPoseIndex));

		var pathPlannerPath = new PathPlannerPath(
			PathPlannerPath.bezierFromPoses(
				drivetrain.getEstimatedPosition(),
				nearestPosition.pose
			),
			PATH_FIND_CONSTRAINTS,
			new GoalEndState(0, nearestPosition.pose.getRotation())
		);

		this.followPathCommand = new FollowPathHolonomic(pathPlannerPath, drivetrain::getEstimatedPosition, drivetrain::getChassisSpeeds, drivetrain::drive, PATH_FIND_FOLLOWER_CONFIG, () -> false, drivetrain);
		this.followPathCommand.initialize();

		this.leds.changeLEDsColor(Colors.LIMELIGHT_GREEN);

		shooter.setRPM(nearestPosition.shooterRPM);
	}

	@Override
	public void execute() {
		if(followPathCommand != null) {
			if(!followPathCommand.isFinished() && !storageIsRunning) {
				followPathCommand.execute();
			} else if(!storageIsRunning && Math.abs(shooter.getRPM() - this.shootingPosition.shooterRPM) <= 50) {
				followPathCommand.end(false);
				storage.setRPM(this.shootingPosition.storageRPM);
				intake.setSpeed(0.75);
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
		if(followPathCommand != null) {
			followPathCommand.end(interrupted);
		}
		storage.setRPM(0);
		shooter.setRPM(0);
		intake.setSpeed(0);
		System.out.println("end " + drivetrain.getEstimatedPosition());
	}
}
