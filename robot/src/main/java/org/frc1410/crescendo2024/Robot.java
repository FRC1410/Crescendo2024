package org.frc1410.crescendo2024;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.frc1410.crescendo2024.commands.*;
import org.frc1410.crescendo2024.commands.ampBarCommands.ExtendAmpBar;
import org.frc1410.crescendo2024.commands.ampBarCommands.ScoreAmp;
import org.frc1410.crescendo2024.commands.drivetrainCommands.AutomaticShooting;
import org.frc1410.crescendo2024.commands.drivetrainCommands.DriveLooped;
//import org.frc1410.crescendo2024.commands.drivetrainCommands.ShootAtNearestPosition;
import org.frc1410.crescendo2024.commands.shooterCommands.Shoot;
import org.frc1410.crescendo2024.commands.shooterCommands.IncrementShooterRPM;
import org.frc1410.crescendo2024.commands.shooterCommands.ShooterManual;
import org.frc1410.crescendo2024.subsystems.*;
import org.frc1410.crescendo2024.util.NetworkTables;
import org.frc1410.framework.AutoSelector;

import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;

public final class Robot extends PhaseDrivenRobot {

	public Robot() {
		NamedCommands.registerCommand("PreloadShoot", new Shoot(shooter, storage, intake,3300, 700));
		NamedCommands.registerCommand("ShooterManual", new ShooterManual(shooter));
		NamedCommands.registerCommand("RunIntakeLimitSwitch", new RunIntakeLimitSwitch(intake, storage, 0.75, 100));
		NamedCommands.registerCommand("RunStorage", new RunStorage(storage, 700));
		NamedCommands.registerCommand("RunIntake", new RunIntake(intake, 0.75));
	}

	private final Controller driverController = new Controller(scheduler, DRIVER_CONTROLLER, 0.1 );
	private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER,  0.1);
	private final Drivetrain drivetrain = subsystems.track(new Drivetrain(subsystems));
	private final Shooter shooter = subsystems.track(new Shooter());
	private final AmpBar ampBar = new AmpBar();
	private final Storage storage = subsystems.track(new Storage());
	private final Intake intake = subsystems.track(new Intake());
	private final LEDs leds = new LEDs();

	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

	private final AutoSelector autoSelector = new AutoSelector()
		.add("Test", () -> new PathPlannerAuto("Test"));

	{
		var profiles = new String[autoSelector.getProfiles().size()];
		for (var i = 0; i < profiles.length; i++) {
			profiles[i] = autoSelector.getProfiles().get(i).name();
		}

		var pub = NetworkTables.PublisherFactory(table, "Choices", profiles);
		pub.accept(profiles);
	}


	private final StringPublisher autoPublisher = NetworkTables.PublisherFactory(table, "Profile",
		autoSelector.getProfiles().isEmpty() ? "" : autoSelector.getProfiles().get(0).name());

	private final StringSubscriber autoSubscriber = NetworkTables.SubscriberFactory(table, autoPublisher.getTopic());

	@Override
	public void autonomousSequence() {

		leds.changeLEDsColor(LEDs.Colors.VIVACIOUS_VIOLENT_VIOLET);

		NetworkTables.SetPersistence(autoPublisher.getTopic(), true);
		String autoProfile = autoSubscriber.get();
		var autoCommand = autoSelector.select(autoProfile);

		scheduler.scheduleAutoCommand(autoCommand);
	}

	@Override
	public void teleopSequence() {
		//leds.defaultLEDsState();

		// Drivetrain
		scheduler.scheduleDefaultCommand(new DriveLooped(drivetrain, driverController.LEFT_X_AXIS, driverController.LEFT_Y_AXIS, driverController.RIGHT_X_AXIS), TaskPersistence.EPHEMERAL);

		driverController.Y.whenPressed(new InstantCommand(
			() -> drivetrain.zeroYaw()
		), TaskPersistence.GAMEPLAY);

		// Shooter
		driverController.LEFT_TRIGGER.button().whileHeld(new ScoreAmp(shooter, storage, false), TaskPersistence.GAMEPLAY);
		driverController.RIGHT_TRIGGER.button().whileHeldOnce(new AutomaticShooting(drivetrain, storage, shooter), TaskPersistence.GAMEPLAY);
		driverController.RIGHT_BUMPER.whileHeld(new ShooterManual(shooter), TaskPersistence.GAMEPLAY);

		driverController.LEFT_BUMPER.whileHeld(new RunStorage(storage, 575), TaskPersistence.GAMEPLAY);
		driverController.LEFT_BUMPER.whileHeld(new RunIntake(intake, 0.5), TaskPersistence.GAMEPLAY);

//		operatorController.RIGHT_BUMPER.whileHeld(new ShooterManual(shooter), TaskPersistence.GAMEPLAY);

		operatorController.RIGHT_BUMPER.whileHeldOnce(new Shoot(shooter, storage, intake,3350,550), TaskPersistence.EPHEMERAL);

		operatorController.RIGHT_TRIGGER.button().whileHeldOnce(new RunIntakeLimitSwitch(intake, storage, INTAKE_SPEED, STORAGE_INTAKE_RPM), TaskPersistence.GAMEPLAY);
		operatorController.LEFT_TRIGGER.button().whileHeld(new Outtake(intake, storage, shooter, OUTTAKE_SPEED, STORAGE_OUTTAKE_SPEED, SHOOTER_OUTTAKE_SPEED), TaskPersistence.GAMEPLAY);

		operatorController.A.whenPressed(new IncrementShooterRPM(shooter, SHOOTER_RPM_INCREMENT), TaskPersistence.GAMEPLAY);
		operatorController.B.whenPressed(new IncrementShooterRPM(shooter, -SHOOTER_RPM_INCREMENT), TaskPersistence.GAMEPLAY);

		operatorController.DPAD_UP.whenPressed(new ExtendAmpBar(ampBar, 0.2, 0.7), TaskPersistence.GAMEPLAY);
		operatorController.DPAD_DOWN.whenPressed(new ExtendAmpBar(ampBar, -0.2, 0.7), TaskPersistence.GAMEPLAY);
	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
