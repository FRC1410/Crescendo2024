package org.frc1410.crescendo2024.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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

    // Subsystems
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    // Misc
    private final SwerveDrivePoseEstimator poseEstimator;

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
            this.gyro.getRotation2d(),
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
            this.gyro.getRotation2d(),
            this.getSwerveModulePositions(),
            pose
        );
    }

    public void zeroYaw() {
        this.gyro.zeroYaw();
		this.resetPose(new Pose2d(this.getEstimatedPosition().getTranslation(), this.gyro.getRotation2d()));
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(
            this.gyro.getRotation2d(),
            this.getSwerveModulePositions()
        );
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }
}
