package org.frc1410.crescendo2024.commands;

import org.frc1410.crescendo2024.commands.intake.RunIntake;
import org.frc1410.crescendo2024.commands.shooter.RunShooter;
import org.frc1410.crescendo2024.commands.shooter.ShootSpeaker;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static org.frc1410.crescendo2024.util.Constants.*;

public class DefensiveAuto extends SequentialCommandGroup {
    public DefensiveAuto(Drivetrain drivetrain, Intake intake, Storage storage, Shooter shooter, Pose2d endPose) {
		var startPath = PathPlannerPath.fromPathFile("dynamic defensive start");

		var startFollowPathCommand = new FollowPathHolonomic(
			startPath,
			drivetrain::getEstimatedPosition, 
			drivetrain::getChassisSpeeds, 
			drivetrain::drive, 
			HOLONOMIC_AUTO_CONFIG,
			() -> false, 
			drivetrain
		);

        var dynamicPath = new PathPlannerPath(
			PathPlannerPath.bezierFromPoses(
				drivetrain.getEstimatedPosition(),
				endPose
			),
			PATH_FOLLOWING_CONSTRAINTS,
			new GoalEndState(0, endPose.getRotation())
		);

		var dynamicFollowPathCommand = new FollowPathHolonomic(
			dynamicPath,
			drivetrain::getEstimatedPosition, 
			drivetrain::getChassisSpeeds, 
			drivetrain::drive, 
			HOLONOMIC_AUTO_CONFIG,
			() -> false, 
			drivetrain
		);

		var endPath = PathPlannerPath.fromPathFile("dynamic defensive end");

		var endFollowPathCommand = new FollowPathHolonomic(
			endPath,
			drivetrain::getEstimatedPosition, 
			drivetrain::getChassisSpeeds, 
			drivetrain::drive, 
			HOLONOMIC_AUTO_CONFIG,
			() -> false, 
			drivetrain
		);

		// keep a count of the number of notes, then run the intake -> shoot command n times and afterwards just intake and then leave.
        this.addCommands(
			startFollowPathCommand,
			new ParallelCommandGroup(
				dynamicFollowPathCommand,
				new RunIntake(intake, INTAKE_SPEED),
				new RunStorage(storage, STORAGE_INTAKE_RPM),
				new RunShooter(shooter, SHOOTER_PLOP_RPM)
			),
			new ParallelRaceGroup(
				new SequentialCommandGroup(
					endFollowPathCommand,
					new ShootSpeaker(storage, intake)
				),
				new RunShooter(shooter, AUTO_SPEAKER_SHOOTER_RPM)
			)
        );
    }
}
