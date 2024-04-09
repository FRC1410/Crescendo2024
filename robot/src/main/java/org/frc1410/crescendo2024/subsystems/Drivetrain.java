package org.frc1410.crescendo2024.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.frc1410.framework.scheduler.subsystem.SubsystemStore;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;
import org.photonvision.EstimatedRobotPose;

import static org.frc1410.crescendo2024.util.IDs.*;

import org.frc1410.crescendo2024.util.NetworkTables;

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

    private final DoublePublisher characterizationVolts = NetworkTables.PublisherFactory(this.table, "characterization volts", 0);

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("pose", Pose2d.struct).publish();

    private final StructPublisher<Pose2d> encoderOnlyPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("encoderOnlyPose", Pose2d.struct).publish();

    // Subsystems
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

	private final Camera camera = new Camera();

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    // Misc
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveDrivePoseEstimator encoderOnlyPoseEstimator;

	private double previousPipelineTimestamp = 0;

	private Rotation2d fieldRelativeOffset = new Rotation2d();

    public Drivetrain(SubsystemStore subsystems) {
		AutoBuilder.configureHolonomic(
			this::getEstimatedPosition,
			this::resetPose,
			this::getChassisSpeeds,
			this::drive,
			HOLONOMIC_AUTO_CONFIG,
			() -> {
				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
			},
			this
		);

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
            this.getGyroYaw(),
            this.getSwerveModulePositions(),
            new Pose2d()
        );

        this.encoderOnlyPoseEstimator = new SwerveDrivePoseEstimator(
            SWERVE_DRIVE_KINEMATICS, 
            this.getGyroYaw(), 
            this.getSwerveModulePositions(), 
            new Pose2d()
        );
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
        Rotation2d angle;
        // TODO: what
        if (DriverStation.getAlliance().isPresent()) {
            angle = DriverStation.getAlliance().get() == Alliance.Blue 
                ? this.getGyroYaw().minus(this.fieldRelativeOffset)
                : this.getGyroYaw().minus(this.fieldRelativeOffset).rotateBy(Rotation2d.fromDegrees(180));
        } else {
            angle = this.getGyroYaw().minus(this.fieldRelativeOffset);
        }

        var robotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, angle);
		this.drive(robotRelativeChassisSpeeds);
    }

    public void driveVolts(double volts) {
        this.characterizationVolts.set(volts);
		frontLeft.driveVolts(volts);
		frontRight.driveVolts(volts);
		backLeft.driveVolts(volts);
		backRight.driveVolts(volts);
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
            this.getGyroYaw(),
            this.getSwerveModulePositions(),
            pose
        );

        this.encoderOnlyPoseEstimator.resetPosition(
            this.getGyroYaw(), 
            this.getSwerveModulePositions(), 
            pose
        );

		this.fieldRelativeOffset = this.getGyroYaw().minus(pose.getRotation());
    }

    public void setYaw(Rotation2d yaw) {
        this.resetPose(new Pose2d(this.getEstimatedPosition().getTranslation(), yaw));
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(
            this.getGyroYaw(),
            this.getSwerveModulePositions()
        );

        // this.encoderOnlyPoseEstimator.update(
        //     this.getGyroYaw(), 
        //     this.getSwerveModulePositions()
        // );

		var estimatedPose = this.camera.getEstimatedPose();

		if(estimatedPose.isPresent() && this.validateVisionPose(estimatedPose.get())) {
		 	var resultTimestamp = estimatedPose.get().timestampSeconds;

            // var b = estimatedPose.get().targetsUsed.stream().filter((elm) -> List.of(7, 8, 3, 4).contains(elm.getFiducialId())).count() >= 1;

		 	if((resultTimestamp != this.previousPipelineTimestamp)) {
                
				this.previousPipelineTimestamp = resultTimestamp;
                System.out.println("Encoder: " + estimatedPose.get().estimatedPose.toPose2d());
				this.poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), resultTimestamp);
                System.out.println("Vision: " + this.getEstimatedPosition());
		 	}
		}

		this.poseX.set(this.getEstimatedPosition().getX());
		this.poseY.set(this.getEstimatedPosition().getY());
		this.heading.set(this.getEstimatedPosition().getRotation().getDegrees());

		this.yaw.set(this.getGyroYaw().getDegrees());
		this.pitch.set(this.gyro.getPitch());
		this.roll.set(this.gyro.getRoll());

        this.posePublisher.set(this.getEstimatedPosition());
        this.encoderOnlyPosePublisher.set(this.encoderOnlyPoseEstimator.getEstimatedPosition());
    }

    private boolean validateVisionPose(EstimatedRobotPose pose) {
        return true;
        
		// var minAmbiguity = pose
		// 	.targetsUsed
		// 	.stream()
		// 	.mapToDouble((target) ->
		// 		target.getPoseAmbiguity()
		// 	)
		// 	.min();

        // var estimatedPosition = this.getEstimatedPosition();

		// var minDistance = pose
		// 	.targetsUsed
		// 	.stream()
		// 	.mapToDouble((target) ->
        //         APRIL_TAG_FIELD_LAYOUT
        //             .getTagPose(target.getFiducialId())
        //             .get()
        //             .getTranslation()
        //             .toTranslation2d()
        //             .getDistance(estimatedPosition.getTranslation())
		// 	)
		// 	.min();

		// if (minAmbiguity.isEmpty() || minDistance.isEmpty()) {
		// 	return false;
		// }

		// return minAmbiguity.getAsDouble() < MAX_APRIL_TAG_AMBIGUITY && minDistance.getAsDouble() < MAX_APRIL_TAG_DISTANCE;
	}

    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

	private Rotation2d getGyroYaw() {
		return Rotation2d.fromDegrees(-this.gyro.getYaw());
	}

    public double getAverageModuleDriveVelocity() {
        return (
            frontLeft.a() +
            frontRight.a() + 
            backLeft.a() +
            backRight.a()
        ) / 4;
    }

    public void alignWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    }

    public void lockWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
    }
}
