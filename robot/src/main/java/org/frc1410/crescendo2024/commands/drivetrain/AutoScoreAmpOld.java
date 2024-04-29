package org.frc1410.crescendo2024.commands.drivetrain;

import org.frc1410.crescendo2024.commands.shooter.RunShooter;
import org.frc1410.crescendo2024.commands.shooter.ShootSpeaker;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static org.frc1410.crescendo2024.util.Constants.*;

import java.util.Optional;

public class AutoScoreAmpOld extends ParallelRaceGroup {
    public AutoScoreAmpOld(Drivetrain drivetrain, Shooter shooter, Storage storage, Intake intake) {
		var ampScoringPosition = (DriverStation.getAlliance() == Optional.of(Alliance.Blue))
			? AMP_SCORING_POSITION_BLUE
			: AMP_SCORING_POSITION_RED;

        var pathPlannerPath = new PathPlannerPath(
			PathPlannerPath.bezierFromPoses(
				drivetrain.getEstimatedPosition(),
				ampScoringPosition
			),
			PATH_FOLLOWING_CONSTRAINTS,
			new GoalEndState(0, ampScoringPosition.getRotation())
		);

		var followPathCommand = new FollowPathHolonomic(
			pathPlannerPath,
			drivetrain::getEstimatedPosition, 
			drivetrain::getChassisSpeeds, 
			drivetrain::drive, 
			HOLONOMIC_PATH_FOLLOWING_CONFIG,
			() -> false, 
			drivetrain
		);

        this.addCommands(
			new RunShooter(shooter, AMP_SHOOTER_VELOCITY, false, null),
			new SequentialCommandGroup(
				followPathCommand,
				new ParallelRaceGroup(
					new LockDrivetrain(drivetrain),
					// new InstantCommand({
						
					// }),
					new WaitCommand(2)
					// new ShootSpeaker(storage, intake)
				)
			)
        );
    }
}
