//package org.frc1410.crescendo2024.commands.drivetrainCommands;
//
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.wpilibj2.command.Command;
//import org.frc1410.crescendo2024.subsystems.Drivetrain;
//import org.frc1410.crescendo2024.subsystems.Shooter;
//import org.frc1410.crescendo2024.subsystems.Storage;
//
//import static org.frc1410.crescendo2024.util.Constants.SHOOTING_POSITIONS;
//
//
//public class ShootAtNearestPosition extends Command {
//	private Drivetrain drivetrain;
//
//	private Shooter shooter;
//
//	private Storage storage;
//
//	private AutomaticShooting command;
//
//	public ShootAtNearestPosition(Drivetrain drivetrain, Shooter shooter, Storage storage) {
//		this.drivetrain = drivetrain;
//		this.shooter = shooter;
//		this.storage = storage;
//		addRequirements();
//	}
//
//	@Override
//	public void initialize() {
//		Pose2d currentRobotPose = drivetrain.getEstimatedPosition();
//		var shootingPoseList = SHOOTING_POSITIONS.stream().map(shootingPositions -> shootingPositions.pose).toList();
//		Pose2d nearestPose = currentRobotPose.nearest(shootingPoseList);
//
//		int nearestPoseIndex = shootingPoseList.indexOf(nearestPose);
//		double shooterRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).shooterRPM;
//		double storageRPM = SHOOTING_POSITIONS.get(nearestPoseIndex).storageRPM;
//
//		System.out.println("current " + currentRobotPose);
//		System.out.println("nearest" + nearestPose);
//
//		this.command = new AutomaticShooting(this.drivetrain, this.shooter, this.storage, SHOOTING_POSITIONS.get(nearestPoseIndex));
//
//		this.command.initialize();
//	}
//
//	@Override
//	public void execute() {
//		if (this.command != null) {
//			this.command.execute();
//		}
//	}
//
//	@Override
//	public boolean isFinished() {
//		if (this.command != null) {
//			boolean a = this.command.isFinished();
//			System.out.println("wrapper is finished: " + a);
//			return a;
//		}
//		return true;
//	}
//
//	@Override
//	public void end(boolean interrupted) {
//		if (this.command != null) {
//			this.command.end(interrupted);
//		}
//	}
//}
