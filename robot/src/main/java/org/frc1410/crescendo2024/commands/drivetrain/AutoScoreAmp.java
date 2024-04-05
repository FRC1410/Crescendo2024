package org.frc1410.crescendo2024.commands.drivetrain;

import static org.frc1410.crescendo2024.util.Constants.AMP_SCORING_POSITION;
import static org.frc1410.crescendo2024.util.Constants.APM_SHOOTER_SPEED;
import static org.frc1410.crescendo2024.util.Constants.HOLONOMIC_PATH_FOLLOWING_CONFIG;
import static org.frc1410.crescendo2024.util.Constants.PATH_FOLLOWING_CONSTRAINTS;

import org.frc1410.crescendo2024.commands.shooter.FireShooter;
import org.frc1410.crescendo2024.commands.shooter.RunShooter;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoScoreAmp extends SequentialCommandGroup {
    public AutoScoreAmp(Drivetrain drivetrain, Shooter shooter, Storage storage, Intake intake) {
        var pathPlannerPath = new PathPlannerPath(
			PathPlannerPath.bezierFromPoses(
				drivetrain.getEstimatedPosition(),
				AMP_SCORING_POSITION
			),
			PATH_FOLLOWING_CONSTRAINTS,
			new GoalEndState(0, AMP_SCORING_POSITION.getRotation())
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
            new ParallelRaceGroup(
                followPathCommand,
                new RunShooter(shooter, APM_SHOOTER_SPEED)
            ),
            new ParallelRaceGroup(
                new WaitCommand(2),
                new FireShooter(storage, intake)
            )
        );
    }
}
