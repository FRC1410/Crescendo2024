package org.frc1410.crescendo2024.commands.drivetrainCommands;

import com.pathplanner.lib.commands.FollowPathHolonomic;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
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

	private ShootingPosition shootingPosition;
	private final Timer timer = new Timer();
	private boolean storageIsRunning = false;
	private LEDs leds;

	private FollowPathHolonomic followPathCommand = null;

	private Pose2d nearestPose;


	public AutomaticShooting(Drivetrain drivetrain, Storage storage, Intake intake, Shooter shooter, LEDs leds) {
		this.drivetrain = drivetrain;
		this.storage = storage;
		this.shooter = shooter;
		this.intake = intake;
		this.leds = leds;
		addRequirements(drivetrain, leds);
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

		this.shootingPosition = SHOOTING_POSITIONS.get(nearestPoseIndex);

		System.out.println("current " + currentRobotPose);
		System.out.println("nearest " + nearestPose);

		var pathPlannerPath = new PathPlannerPath(
			PathPlannerPath.bezierFromPoses(
				drivetrain.getEstimatedPosition(),
				nearestPose
			),
			PATH_FIND_CONSTRAINTS,
			new GoalEndState(0, nearestPose.getRotation())
		);

		leds.changeLEDsColor(LEDs.Colors.LIMELIGHT_GREEN);
		this.followPathCommand = new FollowPathHolonomic(pathPlannerPath, drivetrain::getEstimatedPosition, drivetrain::getChassisSpeeds, drivetrain::drive, PATH_FIND_FOLLOWER_CONFIG, () -> false, drivetrain);
		this.followPathCommand.initialize();
		shooter.setRPM(this.shootingPosition.shooterRPM);
	}

	@Override
	public void execute() {
		System.out.println(shooter.getRPM());
		if(followPathCommand != null) {
			if(!followPathCommand.isFinished() && !storageIsRunning) {
				followPathCommand.execute();
			} else if (!storageIsRunning) {
				drivetrain.lockDrivetrain();
				
				if(Math.abs(shooter.getRPM() - this.shootingPosition.shooterRPM) <= 50) {
					System.out.println("Correct RPM");
					followPathCommand.end(false);
					storage.setRPM(this.shootingPosition.storageRPM);
					intake.setSpeed(0.75);
					storageIsRunning = true;
					timer.start();
				}
			}
		}
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(1);
	}

	@Override
	public void end(boolean interrupted) {
		if(followPathCommand != null) {
			followPathCommand.end(interrupted);
			leds.defaultLEDsState();
		}
		storage.setRPM(0);
		shooter.setRPM(0);
		intake.setSpeed(0);
	}
}
