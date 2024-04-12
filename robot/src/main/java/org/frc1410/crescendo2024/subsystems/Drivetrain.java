package org.frc1410.crescendo2024.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.frc1410.framework.scheduler.subsystem.SubsystemStore;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;
import org.photonvision.EstimatedRobotPose;

import static org.frc1410.crescendo2024.util.IDs.*;

import java.util.Optional;

import org.frc1410.crescendo2024.util.NetworkTables;

import static edu.wpi.first.units.Units.Volts;
import static org.frc1410.crescendo2024.util.Constants.*;

public class Drivetrain implements TickedSubsystem {
    // Network tables
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

    private final DoublePublisher frontLeftVelocitySetpoint = NetworkTables.PublisherFactory(this.table, "Front Left Velocity Setpoint", 0);
    private final DoublePublisher frontRightVelocitySetpoint = NetworkTables.PublisherFactory(this.table, "Front Right Velocity Setpoint", 0);
    private final DoublePublisher backLeftVelocitySetpoint = NetworkTables.PublisherFactory(this.table, "Back Left Velocity Setpoint", 0);
    private final DoublePublisher backRightVelocitySetpoint = NetworkTables.PublisherFactory(this.table, "Back Right Velocity Setpoint", 0);

    private final DoublePublisher frontLeftAngleSetpoint = NetworkTables.PublisherFactory(this.table, "Front Left Angle Setpoint", 0);
    private final DoublePublisher frontRightAngleSetpoint = NetworkTables.PublisherFactory(this.table, "Front Right Angle Setpoint", 0);
    private final DoublePublisher backLeftAngleSetpoint = NetworkTables.PublisherFactory(this.table, "Back Left Angle Setpoint", 0);
    private final DoublePublisher backRightAngleSetpoint = NetworkTables.PublisherFactory(this.table, "Back Right Angle Setpoint", 0);

    private final DoublePublisher frontLeftObservedVelocity = NetworkTables.PublisherFactory(this.table, "Front Left Observed Velocity", 0);
    private final DoublePublisher frontRightObservedVelocity = NetworkTables.PublisherFactory(this.table, "Front Right Observed Velocity", 0);
    private final DoublePublisher backLeftObservedVelocity = NetworkTables.PublisherFactory(this.table, "Back Left Observed Velocity", 0);
    private final DoublePublisher backRightObservedVelocity = NetworkTables.PublisherFactory(this.table, "Back Right Observed Velocity", 0);

    private final DoublePublisher frontLeftObservedAngle = NetworkTables.PublisherFactory(this.table, "Front Left Observed Angle", 0);
    private final DoublePublisher frontRightObservedAngle = NetworkTables.PublisherFactory(this.table, "Front Right Observed Angle", 0);
    private final DoublePublisher backLeftObservedAngle = NetworkTables.PublisherFactory(this.table, "Back Left Observed Angle", 0);
    private final DoublePublisher backRightObservedAngle = NetworkTables.PublisherFactory(this.table, "Back Right Observed Angle", 0);

    private final DoublePublisher poseX = NetworkTables.PublisherFactory(this.table, "X position", 0);
    private final DoublePublisher poseY = NetworkTables.PublisherFactory(this.table, "y position", 0);
    private final DoublePublisher heading = NetworkTables.PublisherFactory(this.table, "Heading", 0);

    private final DoublePublisher yaw = NetworkTables.PublisherFactory(this.table, "yaw", 0);
    private final DoublePublisher pitch = NetworkTables.PublisherFactory(this.table, "pitch", 0);
    private final DoublePublisher roll = NetworkTables.PublisherFactory(this.table, "roll", 0);

    private final DoublePublisher characterizationVolts = NetworkTables.PublisherFactory(this.table,
            "characterization volts", 0);

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
            this.frontLeftVelocitySetpoint,
            this.frontLeftAngleSetpoint,
            this.frontLeftObservedVelocity,
            this.frontLeftObservedAngle
        ));

        this.frontRight = subsystems.track(new SwerveModule(
            FRONT_RIGHT_DRIVE_MOTOR,
            FRONT_RIGHT_STEER_MOTOR,
            FRONT_RIGHT_STEER_ENCODER,
            FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
            FRONT_RIGHT_STEER_MOTOR_INVERTED,
            FRONT_RIGHT_STEER_ENCODER_OFFSET,
            this.frontRightVelocitySetpoint,
            this.frontRightAngleSetpoint,
            this.frontRightObservedVelocity,
            this.frontRightObservedAngle
        ));

        this.backLeft = subsystems.track(new SwerveModule(
            BACK_LEFT_DRIVE_MOTOR,
            BACK_LEFT_STEER_MOTOR,
            BACK_LEFT_STEER_ENCODER,
            BACK_LEFT_DRIVE_MOTOR_INVERTED,
            BACK_LEFT_STEER_MOTOR_INVERTED,
            BACK_LEFT_STEER_ENCODER_OFFSET,
            this.backLeftVelocitySetpoint,
            this.backLeftAngleSetpoint,
            this.backLeftObservedVelocity,
            this.backLeftObservedAngle
        ));

        this.backRight = subsystems.track(new SwerveModule(
            BACK_RIGHT_DRIVE_MOTOR,
            BACK_RIGHT_STEER_MOTOR,
            BACK_RIGHT_STEER_ENCODER,
            BACK_RIGHT_DRIVE_MOTOR_INVERTED,
            BACK_RIGHT_STEER_MOTOR_INVERTED,
            BACK_RIGHT_STEER_ENCODER_OFFSET,
            this.backRightVelocitySetpoint,
            this.backRightAngleSetpoint,
            this.backRightObservedVelocity,
            this.backRightObservedAngle
        ));

        this.poseEstimator = new SwerveDrivePoseEstimator(
            SWERVE_DRIVE_KINEMATICS,
            this.getGyroYaw(),
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
        Rotation2d angle = this.getGyroYaw().minus(this.fieldRelativeOffset);
        if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            angle = angle.rotateBy(Rotation2d.fromDegrees(180));
        }

        var robotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, angle);

        this.drive(robotRelativeChassisSpeeds);
    }

    public void drive(Measure<Voltage> voltage) {
        this.characterizationVolts.set(voltage.in(Volts));
        
		frontLeft.drive(voltage);
		frontRight.drive(voltage);
		backLeft.drive(voltage);
		backRight.drive(voltage);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(
                this.frontLeft.getState(),
                this.frontRight.getState(),
                this.backLeft.getState(),
                this.backRight.getState());
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

        this.fieldRelativeOffset = this.getGyroYaw().minus(pose.getRotation());
    }

    public void setYaw(Rotation2d yaw) {
        this.resetPose(new Pose2d(this.getEstimatedPosition().getTranslation(), yaw));
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(
                this.getGyroYaw(),
                this.getSwerveModulePositions());

        var estimatedPose = this.camera.getEstimatedPose();

        if (estimatedPose.isPresent() && this.validateVisionPose(estimatedPose.get())) {
            var resultTimestamp = estimatedPose.get().timestampSeconds;

            if ((resultTimestamp != this.previousPipelineTimestamp)) {

                this.previousPipelineTimestamp = resultTimestamp;
                this.poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), resultTimestamp);
            }
        }

        this.poseX.set(this.getEstimatedPosition().getX());
        this.poseY.set(this.getEstimatedPosition().getY());
        this.heading.set(this.getEstimatedPosition().getRotation().getDegrees());

        this.yaw.set(this.getGyroYaw().getDegrees());
        this.pitch.set(this.gyro.getPitch());
        this.roll.set(this.gyro.getRoll());

        this.posePublisher.set(this.getEstimatedPosition());
        this.encoderOnlyPosePublisher.set(new Pose2d(new Translation2d(4, 4), this.getGyroYaw()));
    }

    private boolean validateVisionPose(EstimatedRobotPose pose) {
        var minAmbiguity = pose.targetsUsed
            .stream()
            .mapToDouble((target) -> target.getPoseAmbiguity())
            .min();

        var estimatedPosition = this.getEstimatedPosition();

        var minDistance = pose.targetsUsed
            .stream()
            .mapToDouble((target) -> APRIL_TAG_FIELD_LAYOUT
                .getTagPose(target.getFiducialId())
                .get()
                .getTranslation()
                .toTranslation2d()
                .getDistance(estimatedPosition.getTranslation())
            )
            .min();

        if (minAmbiguity.isEmpty() || minDistance.isEmpty()) {
            return false;
        }

        return minAmbiguity.getAsDouble() < MAX_APRIL_TAG_AMBIGUITY && minDistance.getAsDouble() < MAX_APRIL_TAG_DISTANCE;
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

    public Measure<Velocity<Angle>> getAverageDriveAngularVelocity() {
        return this.frontLeft.getDriveAngularVelocity()
            .plus(this.frontRight.getDriveAngularVelocity())
            .plus(this.backLeft.getDriveAngularVelocity())
            .plus(this.backRight.getDriveAngularVelocity())
            .divide(4);
    }

    public void lockWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
    }
}
