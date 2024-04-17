package org.frc1410.crescendo2024.commands.drivetrain;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.LEDs;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;
import org.frc1410.crescendo2024.subsystems.LEDs.Color;
import org.frc1410.crescendo2024.util.ShootingPosition;

import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static org.frc1410.crescendo2024.util.Constants.*;

import java.util.Optional;

public class AutoScoreSpeaker extends Command {
	private final Drivetrain drivetrain;
	private final Shooter shooter;
	private final Storage storage;
	private final Intake intake;
	private final LEDs leds;

	private ShootingPosition shootingPosition;

	private boolean storageIsRunning = false;

	private final Timer shootingTimer = new Timer();

	private FollowPathHolonomic followPathCommand = null;

	public AutoScoreSpeaker(Drivetrain drivetrain, Shooter shooter, Storage storage, Intake intake, LEDs leds) {
		this.drivetrain = drivetrain;
		this.shooter = shooter;
		this.storage = storage;
		this.intake = intake;
		this.leds = leds;

		this.addRequirements(drivetrain, shooter, storage, intake, leds);
	}

	@Override
	public void initialize() {
		var shootingPositions = DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))
			? SHOOTING_POSITIONS_BLUE
			: SHOOTING_POSITIONS_RED;
		
		this.shootingTimer.stop();
		this.shootingTimer.reset();

		this.storageIsRunning = false;

		Pose2d currentRobotPose = this.drivetrain.getEstimatedPosition();
		
		ShootingPosition nearestPosition = null;
		double smallestDistance = Double.MAX_VALUE;
		
		for (ShootingPosition position : shootingPositions) {
			var distance = position.pose.getTranslation().getDistance(currentRobotPose.getTranslation());
			if (distance < smallestDistance) {
				smallestDistance = distance;
				nearestPosition = position;
			}
		}

		this.shootingPosition = nearestPosition;

		var pathPlannerPath = new PathPlannerPath(
			PathPlannerPath.bezierFromPoses(
				this.drivetrain.getEstimatedPosition(),
				nearestPosition.pose
			),
			PATH_FOLLOWING_CONSTRAINTS,
			new GoalEndState(0, nearestPosition.pose.getRotation())
		);

		this.followPathCommand = new FollowPathHolonomic(
			pathPlannerPath,
			drivetrain::getEstimatedPosition, 
			drivetrain::getChassisSpeeds, 
			drivetrain::drive, 
			HOLONOMIC_PATH_FOLLOWING_CONFIG,
			() -> false, 
			drivetrain
		);

		this.followPathCommand.initialize();

		this.leds.setColor(Color.LIMELIGHT_GREEN);

		this.shooter.setVelocity(nearestPosition.shooterVelocity);
	}

	@Override
	public void execute() {
		if(!this.followPathCommand.isFinished() && !this.storageIsRunning) {
			// If not at goal pose, continue following path
			if(this.followPathCommand != null) {
				this.followPathCommand.execute();
			}
		} else if(!this.storageIsRunning) {
			// Else, wait until shooter is spun up and then shoot
			if(this.followPathCommand != null) {
				this.followPathCommand.end(false);
				this.followPathCommand = null;
			}

			this.drivetrain.lockWheels();

			if (Math.abs(this.shooter.getVelocity().in(RPM) - this.shootingPosition.shooterVelocity.in(RPM)) < 100) {
				this.storage.setVelocity(this.shootingPosition.storageVelocity);
				this.intake.setSpeed(0.75);
				this.storageIsRunning = true;
				this.shootingTimer.start();
			}
		}
	}

	@Override
	public boolean isFinished() {
		return this.shootingTimer.hasElapsed(SHOOTING_TIME.in(Seconds));
	}

	@Override
	public void end(boolean interrupted) {
		if(this.followPathCommand != null) {
			followPathCommand.end(interrupted);
		}

		this.storage.setVelocity(RPM.zero());
		this.shooter.setVelocity(RPM.zero());
		this.intake.setSpeed(0);
	}
}