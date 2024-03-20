package org.frc1410.crescendo2024;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.frc1410.crescendo2024.commands.ClimbLooped;
import org.frc1410.crescendo2024.commands.RunStorage;
import org.frc1410.crescendo2024.commands.Intake.FlipIntake;
import org.frc1410.crescendo2024.commands.Intake.OuttakeNote;
import org.frc1410.crescendo2024.commands.Intake.SetIntakeStateLEDColor;
import org.frc1410.crescendo2024.commands.Intake.IntakeNote;
import org.frc1410.crescendo2024.commands.drivetrain.AutoScoreSpeaker;
import org.frc1410.crescendo2024.commands.drivetrain.DriveLooped;
import org.frc1410.crescendo2024.commands.shooterCommands.ShootNote;
import org.frc1410.crescendo2024.subsystems.Climber;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.LEDs;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;
import org.frc1410.crescendo2024.commands.shooterCommands.AdjustShooterRPM;
import org.frc1410.crescendo2024.commands.shooterCommands.FireShooter;
import org.frc1410.crescendo2024.commands.shooterCommands.RunShooter;
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
		NamedCommands.registerCommand("ShootNote", new ShootNote(drivetrain, shooter, storage, intake, leds, AUTO_SPEAKER_SHOOTER_RPM, AUTO_SPEAKER_STORAGE_RPM));
		NamedCommands.registerCommand("RunShooter", new RunShooter(shooter, AUTO_SPEAKER_SHOOTER_RPM));
		NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, storage));
		NamedCommands.registerCommand("FireShooter", new FireShooter(storage, intake));
		NamedCommands.registerCommand("FlipIntake", new FlipIntake(intake));
	}

	private final Controller driverController = new Controller(scheduler, DRIVER_CONTROLLER, 0.1 );
	private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER,  0.1);

	private final Drivetrain drivetrain = subsystems.track(new Drivetrain(subsystems));
	private final Shooter shooter = subsystems.track(new Shooter());
	private final Storage storage = subsystems.track(new Storage());
	private final Intake intake = subsystems.track(new Intake());
	private final Climber climb = new Climber();
	private final LEDs leds = new LEDs();

	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

	private final AutoSelector autoSelector = new AutoSelector()
		.add("0", () -> new InstantCommand())
		.add("1", () -> new ShootNote(drivetrain, shooter, storage, intake, leds, AUTO_SPEAKER_SHOOTER_RPM, AUTO_SPEAKER_STORAGE_RPM))
		.add("1 intake",() -> new PathPlannerAuto("1 piece intake"))
		.add("1 drive", () -> new PathPlannerAuto("1.5 source side auto"))
		.add("3", () -> new PathPlannerAuto("3 piece mid sub"))
		.add("3 amp",() -> new PathPlannerAuto("3 piece amp side auto"))
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
		this.leds.changeLEDsColor(LEDs.Color.VIVACIOUS_VIOLENT_VIOLET);

		NetworkTables.SetPersistence(this.autoPublisher.getTopic(), true);
		String autoProfile = this.autoSubscriber.get();
		var autoCommand = this.autoSelector.select(autoProfile);

		this.scheduler.scheduleAutoCommand(autoCommand);
	}

	@Override
	public void teleopSequence() {
		// Drivetrain
		scheduler.scheduleDefaultCommand(new DriveLooped(
			drivetrain, 
			driverController.LEFT_X_AXIS, 
			driverController.LEFT_Y_AXIS, 
			driverController.RIGHT_X_AXIS, 
			driverController.LEFT_TRIGGER
		), TaskPersistence.EPHEMERAL);

		driverController.Y.whenPressed(new InstantCommand(
			() -> drivetrain.zeroYaw()
		), TaskPersistence.GAMEPLAY);

		// Shooter
		driverController.RIGHT_TRIGGER.button().whileHeldOnce(new AutoScoreSpeaker(
			drivetrain, 
			shooter, 
			storage, 
			intake, 
			leds
		), TaskPersistence.GAMEPLAY, LockPriority.HIGHEST);

		driverController.RIGHT_BUMPER.whileHeld(new RunShooter(shooter, MANUAL_SHOOTER_RPM), TaskPersistence.GAMEPLAY);
		driverController.LEFT_BUMPER.whileHeld(new FireShooter(storage, intake), TaskPersistence.GAMEPLAY);

		operatorController.RIGHT_BUMPER.whileHeldOnce(new RunStorage(storage, MANUAL_STORAGE_RPM), TaskPersistence.GAMEPLAY);
		operatorController.LEFT_BUMPER.whileHeldOnce(new RunShooter(shooter, MANUAL_SHOOTER_RPM), TaskPersistence.GAMEPLAY);

		operatorController.LEFT_TRIGGER.button().whileHeld(new OuttakeNote(intake, storage, shooter), TaskPersistence.GAMEPLAY);

		operatorController.A.whenPressed(new AdjustShooterRPM(shooter, SHOOTER_RPM_ADJUSTMENT_MAGNITUDE), TaskPersistence.GAMEPLAY);
		operatorController.B.whenPressed(new AdjustShooterRPM(shooter, -SHOOTER_RPM_ADJUSTMENT_MAGNITUDE), TaskPersistence.GAMEPLAY);

		// Intake
		operatorController.RIGHT_TRIGGER.button().whileHeldOnce(new IntakeNote(intake, storage), TaskPersistence.GAMEPLAY);

		operatorController.X.whenPressed(new FlipIntake(intake), TaskPersistence.GAMEPLAY);

		scheduler.scheduleDefaultCommand(new SetIntakeStateLEDColor(intake, leds), TaskPersistence.EPHEMERAL);

		// Climber
		scheduler.scheduleDefaultCommand(new ClimbLooped(climb, operatorController.LEFT_Y_AXIS, operatorController.RIGHT_Y_AXIS), TaskPersistence.EPHEMERAL);
	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
