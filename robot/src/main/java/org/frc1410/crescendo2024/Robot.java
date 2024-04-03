package org.frc1410.crescendo2024;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.frc1410.crescendo2024.commands.ClimbLooped;
import org.frc1410.crescendo2024.commands.RunStorage;
import org.frc1410.crescendo2024.commands.drivetrain.AutoScoreSpeaker;
import org.frc1410.crescendo2024.commands.drivetrain.DriveLooped;
import org.frc1410.crescendo2024.commands.intake.FlipIntake;
import org.frc1410.crescendo2024.commands.intake.IntakeNote;
import org.frc1410.crescendo2024.commands.intake.OuttakeNote;
import org.frc1410.crescendo2024.commands.intake.SetIntakeStateLEDColor;
import org.frc1410.crescendo2024.commands.shooter.AdjustShooterRPM;
import org.frc1410.crescendo2024.commands.shooter.FireShooter;
import org.frc1410.crescendo2024.commands.shooter.RunShooter;
import org.frc1410.crescendo2024.commands.shooter.ShootNote;
import org.frc1410.crescendo2024.subsystems.Climber;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.LEDs;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;
import org.frc1410.crescendo2024.util.NetworkTables;

import org.frc1410.framework.AutoSelector;
import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;
import org.frc1410.framework.scheduler.task.lock.LockPriority;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;

public final class Robot extends PhaseDrivenRobot {
	public Robot() {
		// TODO: re-enable
		// DataLogManager.start();

		NamedCommands.registerCommand("ShootNote", new ShootNote(
			this.drivetrain, 
			this.shooter, 
			this.storage, 
			this.intake, 
			this.leds, 
			AUTO_SPEAKER_SHOOTER_RPM, 
			AUTO_SPEAKER_STORAGE_RPM
		));
		NamedCommands.registerCommand("RunShooter", new RunShooter(this.shooter, AUTO_SPEAKER_SHOOTER_RPM));
		NamedCommands.registerCommand("IntakeNote", new IntakeNote(this.intake, this.storage, this.driverController, this.operatorController));
		NamedCommands.registerCommand("FireShooter", new FireShooter(this.storage, this.intake));
		NamedCommands.registerCommand("FlipIntake", new FlipIntake(this.intake));
	}

	private final Controller driverController = new Controller(this.scheduler, DRIVER_CONTROLLER, 0.1);
	private final Controller operatorController = new Controller(this.scheduler, OPERATOR_CONTROLLER,  0.1);

	private final Drivetrain drivetrain = subsystems.track(new Drivetrain(this.subsystems));
	private final Shooter shooter = subsystems.track(new Shooter());
	private final Storage storage = subsystems.track(new Storage());
	private final Intake intake = subsystems.track(new Intake());
	private final Climber climer = new Climber();
	private final LEDs leds = new LEDs();

	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = this.nt.getTable("Auto");

	private final AutoSelector autoSelector = new AutoSelector()
		.add("0", () -> new InstantCommand())
		.add("1", () -> new ShootNote(
			this.drivetrain, 
			this.shooter,
			this.storage, 
			this.intake, 
			this.leds, 
			AUTO_SPEAKER_SHOOTER_RPM, 
			AUTO_SPEAKER_STORAGE_RPM
		))
		.add("1 intake",() -> new PathPlannerAuto("1 piece intake"))
		.add("1 drive", () -> new PathPlannerAuto("1.5 source side auto"))
		.add("2 source", () -> new PathPlannerAuto("2 piece source side auto"))
		.add("3", () -> new PathPlannerAuto("3 piece mid sub"))
		.add("3 amp",() -> new PathPlannerAuto("3 piece amp side auto"))
		.add("3 source", () -> new PathPlannerAuto("3 piece source side auto"))
		.add("4", () -> new PathPlannerAuto("4 piece mid sub"))
		.add("5", () -> new PathPlannerAuto("5 piece mid sub"));

	{
		var profiles = new String[this.autoSelector.getProfiles().size()];
		for (var i = 0; i < profiles.length; i++) {
			profiles[i] = this.autoSelector.getProfiles().get(i).name();
		}

		var pub = NetworkTables.PublisherFactory(this.table, "Choices", profiles);
		pub.accept(profiles);
	}

	private final StringPublisher autoPublisher = NetworkTables.PublisherFactory(this.table, "Profile",
		this.autoSelector.getProfiles().isEmpty() ? "" : this.autoSelector.getProfiles().get(0).name());

	private final StringSubscriber autoSubscriber = NetworkTables.SubscriberFactory(this.table, this.autoPublisher.getTopic());

	@Override
	public void autonomousSequence() {
		this.leds.setColor(LEDs.Color.VIVACIOUS_VIOLENT_VIOLET);

		NetworkTables.SetPersistence(this.autoPublisher.getTopic(), true);
		String autoProfile = this.autoSubscriber.get();
		var autoCommand = this.autoSelector.select(autoProfile);

		this.scheduler.scheduleAutoCommand(autoCommand);

		// var file = new File(Filesystem.getDeployDirectory(), "Unknown.png");

		// if (!file.exists()) {
		// 	System.exit(0);
		// }
	}

	@Override
	public void teleopSequence() {
		// Drivetrain
		this.scheduler.scheduleDefaultCommand(new DriveLooped(
			this.drivetrain, 
			this.driverController.LEFT_X_AXIS, 
			this.driverController.LEFT_Y_AXIS, 
			this.driverController.RIGHT_X_AXIS, 
			this.driverController.LEFT_TRIGGER
		), TaskPersistence.EPHEMERAL);

		this.driverController.Y.whenPressed(new InstantCommand(
			() -> {
				if (DriverStation.getAlliance().get() == Alliance.Blue) {
					this.drivetrain.setYaw(Rotation2d.fromDegrees(180));
				} else {
					this.drivetrain.setYaw(Rotation2d.fromDegrees(0));
				}
			}
		), TaskPersistence.GAMEPLAY);

		// Shooter
		this.driverController.RIGHT_TRIGGER.button().whileHeldOnce(new AutoScoreSpeaker(
			this.drivetrain, 
			this.shooter, 
			this.storage, 
			this.intake, 
			this.leds
		), TaskPersistence.GAMEPLAY, LockPriority.HIGHEST);

		this.driverController.RIGHT_BUMPER.whileHeld(new RunShooter(this.shooter, MANUAL_SHOOTER_RPM), TaskPersistence.GAMEPLAY);
		this.driverController.LEFT_BUMPER.whileHeld(new FireShooter(this.storage, this.intake), TaskPersistence.GAMEPLAY);

		this.operatorController.RIGHT_BUMPER.whileHeld(new RunStorage(this.storage, MANUAL_STORAGE_RPM), TaskPersistence.GAMEPLAY);
		this.operatorController.Y.whileHeld(new RunShooter(this.shooter, APM_SHOOTER_RPM), TaskPersistence.GAMEPLAY);
		this.operatorController.LEFT_BUMPER.whileHeld(new RunShooter(this.shooter, MANUAL_SHOOTER_RPM), TaskPersistence.GAMEPLAY);

		this.operatorController.LEFT_TRIGGER.button().whileHeld(new OuttakeNote(this.intake, this.storage, this.shooter), TaskPersistence.GAMEPLAY);

		this.operatorController.A.whenPressed(new AdjustShooterRPM(this.shooter, SHOOTER_RPM_ADJUSTMENT_MAGNITUDE), TaskPersistence.GAMEPLAY);
		this.operatorController.B.whenPressed(new AdjustShooterRPM(this.shooter, -SHOOTER_RPM_ADJUSTMENT_MAGNITUDE), TaskPersistence.GAMEPLAY);

		// Intake
		this.operatorController.RIGHT_TRIGGER.button().whileHeldOnce(new IntakeNote(this.intake, this.storage, this.driverController, this.operatorController), TaskPersistence.GAMEPLAY);

		this.operatorController.X.whenPressed(new FlipIntake(this.intake), TaskPersistence.GAMEPLAY);

		this.operatorController.START.whenPressed(new InstantCommand(
			() -> this.intake.zeroBarEncoder()
		), TaskPersistence.GAMEPLAY);

		this.scheduler.scheduleDefaultCommand(new SetIntakeStateLEDColor(this.intake, this.leds), TaskPersistence.EPHEMERAL);

		// Climber
		this.scheduler.scheduleDefaultCommand(new ClimbLooped(
			this.climer, 
			this.operatorController.LEFT_Y_AXIS, 
			this.operatorController.RIGHT_Y_AXIS
		), TaskPersistence.EPHEMERAL);
	}

	@Override
	public void testSequence() {
	}

	@Override
	protected void disabledSequence() {

	}
}