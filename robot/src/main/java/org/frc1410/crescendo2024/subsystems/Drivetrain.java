package org.frc1410.crescendo2024.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

import org.frc1410.framework.scheduler.subsystem.SubsystemStore;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.crescendo2024.util.IDs.*;

import org.frc1410.crescendo2024.util.NetworkTables;

import java.util.Optional;

import static org.frc1410.crescendo2024.util.Constants.*;

public class Drivetrain implements TickedSubsystem {
    // Network tables
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

    private final DoublePublisher frontLeftDesiredVel = NetworkTables.PublisherFactory(this.table, "frontLeft Desired Vel", 0);
    private final DoublePublisher frontRightDesiredVel = NetworkTables.PublisherFactory(this.table, "frontRight Desired Vel", 0);
    private final DoublePublisher backLeftDesiredVel = NetworkTables.PublisherFactory(this.table, "backLeft Desired Vel", 0);
    private final DoublePublisher backRightDesiredVel = NetworkTables.PublisherFactory(this.table, "backRight Desired Vel", 0);

    private final DoublePublisher frontLeftDesiredAngle = NetworkTables.PublisherFactory(this.table, "frontLeft Desired angle", 0);
    private final DoublePublisher frontRightDesiredAngle = NetworkTables.PublisherFactory(this.table, "frontRight Desired angle", 0);
    private final DoublePublisher backLeftDesiredAngle = NetworkTables.PublisherFactory(this.table, "backLeft Desired angle", 0);
    private final DoublePublisher backRightDesiredAngle = NetworkTables.PublisherFactory(this.table, "backRight Desired angle", 0);

    private final DoublePublisher frontLeftActualVel = NetworkTables.PublisherFactory(this.table, "frontLeft Actual Vel", 0);
    private final DoublePublisher frontRightActualVel = NetworkTables.PublisherFactory(this.table, "frontRight Actual Vel", 0);
    private final DoublePublisher backLeftActualVel = NetworkTables.PublisherFactory(this.table, "backLeft Actual Vel", 0);
    private final DoublePublisher backRightActualVel = NetworkTables.PublisherFactory(this.table, "backRight Actual Vel", 0);

    private final DoublePublisher frontLeftActualAngle = NetworkTables.PublisherFactory(this.table, "frontLeft Actual angle", 0);
    private final DoublePublisher frontRightActualAngle = NetworkTables.PublisherFactory(this.table, "frontRight Actual angle", 0);
    private final DoublePublisher backLeftActualAngle = NetworkTables.PublisherFactory(this.table, "backLeft Actual angle", 0);
    private final DoublePublisher backRightActualAngle = NetworkTables.PublisherFactory(this.table, "backRight Actual angle", 0);

	private final DoublePublisher poseX = NetworkTables.PublisherFactory(this.table, "X position", 0);
	private final DoublePublisher poseY = NetworkTables.PublisherFactory(this.table, "y position", 0);
	private final DoublePublisher heading = NetworkTables.PublisherFactory(this.table, "Heading", 0);

    private final DoublePublisher yaw = NetworkTables.PublisherFactory(this.table, "yaw", 0);
    private final DoublePublisher pitch = NetworkTables.PublisherFactory(this.table, "pitch", 0);
    private final DoublePublisher roll = NetworkTables.PublisherFactory(this.table, "roll", 0);

    // Subsystems
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

	 private final Camera camera = new Camera();

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Misc
    private final SwerveDrivePoseEstimator poseEstimator;

	private double previousPipelineTimestamp = 0;
    public Drivetrain(SubsystemStore subsystems) {
        this.frontLeft = subsystems.track(new SwerveModule(
            FRONT_LEFT_DRIVE_MOTOR,
            FRONT_LEFT_STEER_MOTOR,
            FRONT_LEFT_STEER_ENCODER,
            FRONT_LEFT_DRIVE_MOTOR_INVERTED,
            FRONT_LEFT_STEER_MOTOR_INVERTED,
            FRONT_LEFT_STEER_ENCODER_OFFSET,
            this.frontLeftDesiredVel,
            this.frontLeftDesiredAngle,
            this.frontLeftActualVel,
            this.frontLeftActualAngle
        ));

        this.frontRight = subsystems.track(new SwerveModule(
            FRONT_RIGHT_DRIVE_MOTOR,
            FRONT_RIGHT_STEER_MOTOR,
            FRONT_RIGHT_STEER_ENCODER,
            FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
            FRONT_RIGHT_STEER_MOTOR_INVERTED,
            FRONT_RIGHT_STEER_ENCODER_OFFSET,
            this.frontRightDesiredVel,
            this.frontRightDesiredAngle,
            this.frontRightActualVel,
            this.frontRightActualAngle
        ));

        this.backLeft = subsystems.track(new SwerveModule(
            BACK_LEFT_DRIVE_MOTOR,
            BACK_LEFT_STEER_MOTOR,
            BACK_LEFT_STEER_ENCODER,
            BACK_LEFT_DRIVE_MOTOR_INVERTED,
            BACK_LEFT_STEER_MOTOR_INVERTED,
            BACK_LEFT_STEER_ENCODER_OFFSET,
            this.backLeftDesiredVel,
            this.backLeftDesiredAngle,
            this.backLeftActualVel,
            this.backLeftActualAngle
        ));

        this.backRight = subsystems.track(new SwerveModule(
            BACK_RIGHT_DRIVE_MOTOR,
            BACK_RIGHT_STEER_MOTOR,
            BACK_RIGHT_STEER_ENCODER,
            BACK_RIGHT_DRIVE_MOTOR_INVERTED,
            BACK_RIGHT_STEER_MOTOR_INVERTED,
            BACK_RIGHT_STEER_ENCODER_OFFSET,
            this.backRightDesiredVel,
            this.backRightDesiredAngle,
            this.backRightActualVel,
            this.backRightActualAngle
        ));

        this.poseEstimator = new SwerveDrivePoseEstimator(
            SWERVE_DRIVE_KINEMATICS,
            this.getAngle().toRotation2d(),
            this.getSwerveModulePositions(),
            new Pose2d()
        );

        this.gyro.reset();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        var swerveModuleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SWERVE_DRIVE_MAX_SPEED);

        this.frontLeft.setDesiredState(swerveModuleStates[0]);
        this.frontRight.setDesiredState(swerveModuleStates[1]);
        this.backLeft.setDesiredState(swerveModuleStates[2]);
        this.backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
        var robotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.getAngle().toRotation2d());
        this.drive(robotRelativeChassisSpeeds);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(
        	this.frontLeft.getState(),
           	this.frontRight.getState(),
           	this.backLeft.getState(),
           	this.backRight.getState()
        );
    }

    public Pose2d getEstimatedPosition() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPosition(
            this.getAngle().toRotation2d(),
            this.getSwerveModulePositions(),
            pose
        );
    }

    public void zeroYaw() {
        this.gyro.zeroYaw();
		this.resetPose(new Pose2d(this.getEstimatedPosition().getTranslation(), this.getAngle().toRotation2d()));
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(
            this.getAngle().toRotation2d(),
            this.getSwerveModulePositions()
        );

//		 var estimatedPose = camera.getEstimatedPose();
//
//		 if(estimatedPose.isPresent()) {
//
//		 	// TODO: Possible bug where bad data is fed into pose estimator when no vision
//		 	var resultTimestamp = estimatedPose.get().timestampSeconds;
//
//		 	if(resultTimestamp != previousPipelineTimestamp) {
//		 		previousPipelineTimestamp = resultTimestamp;
//		 		poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), resultTimestamp);
//		 	}
//
//		 }

		var pipelineResult = camera.getLatestResult();
		var resultTimestamp = pipelineResult.getTimestampSeconds();

		if(resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
			previousPipelineTimestamp = resultTimestamp;
			var target = pipelineResult.getBestTarget();

			var fiducialId = target.getFiducialId();

			Optional<Pose3d> tagPose =
				camera.aprilTagFieldLayout() == null
					? Optional.empty()
					: camera.aprilTagFieldLayout().getTagPose(fiducialId);

			if (target.getPoseAmbiguity() <= 0.2 && fiducialId >= 0 && tagPose.isPresent()) {
				var targetPose = tagPose.get();
				Transform3d camToTarget = target.getBestCameraToTarget();

				System.out.println(camToTarget);

				Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

				var visionMesurement = camPose.transformBy(CAMERAPOSE);
				poseEstimator.addVisionMeasurement(visionMesurement.toPose2d(), resultTimestamp);
			}
		}

//        this.yaw.set(this.gyro.getYaw());
//        this.roll.set(this.gyro.getRoll());
//        this.pitch.set(this.gyro.getPitch());

		poseX.set(this.getEstimatedPosition().getX());
		poseY.set(this.getEstimatedPosition().getY());
		heading.set(this.getEstimatedPosition().getRotation().getDegrees());


    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    private Rotation3d getAngle() {
        return gyro.getRotation3d().rotateBy(NAVX_ANGLE);
    }
}
