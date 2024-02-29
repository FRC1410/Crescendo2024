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
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.SerialPort;
import org.frc1410.framework.scheduler.subsystem.SubsystemStore;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

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

    // Subsystems
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

	private final Camera camera = new Camera();

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

	private LEDs leds = new LEDs();

    // Misc
    private final SwerveDrivePoseEstimator poseEstimator;

	private double previousPipelineTimestamp = 0;

	public boolean teleopIsFieldRelative = true;

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
        var robotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.getGyroYaw().minus(this.fieldRelativeOffset));
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
        System.out.println("RESET POSE TO: " + pose);
        this.poseEstimator.resetPosition(
            this.getGyroYaw(),
            this.getSwerveModulePositions(),
            pose
        );

		this.fieldRelativeOffset = this.getGyroYaw().minus(pose.getRotation());
    }

    public void zeroYaw() {
        this.resetPose(new Pose2d(this.getEstimatedPosition().getTranslation(), new Rotation2d()));
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(
            this.getGyroYaw(),
            this.getSwerveModulePositions()
        );

		var estimatedPose = camera.getEstimatedPose();

		if(estimatedPose.isPresent()) {
		 	var resultTimestamp = estimatedPose.get().timestampSeconds;

		 	if(resultTimestamp != previousPipelineTimestamp) {
				previousPipelineTimestamp = resultTimestamp;
				poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), resultTimestamp);
		 	}
		}

		poseX.set(this.getEstimatedPosition().getX());
		poseY.set(this.getEstimatedPosition().getY());
		heading.set(this.getEstimatedPosition().getRotation().getDegrees());

		yaw.set(this.getGyroYaw().getDegrees());
		pitch.set(this.gyro.getPitch());
		roll.set(this.gyro.getRoll());
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

    public void alignWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    }
}
