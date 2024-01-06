package org.frc1410.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import org.frc1410.framework.AutoSelector;
import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.test.util.NetworkTables;

import static org.frc1410.test.util.Constants.*;

public final class Robot extends PhaseDrivenRobot {

    private final Controller driverController = new Controller(scheduler, DRIVER_CONTROLLER, 0.12);
    private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER, 0.25);

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable table = nt.getTable("Auto");

    private final AutoSelector autoSelector = new AutoSelector();

    private final StringPublisher autoPublisher = NetworkTables.PublisherFactory(table, "Profile",
            autoSelector.getProfiles().get(0).name());
    private final StringSubscriber autoSubscriber = NetworkTables.SubscriberFactory(table, autoPublisher.getTopic());

    public Robot() {

    }

	@Override
	public void disabledSequence() {

	}

//    @Override
//    public void autonomousSequence() {
//
//        NetworkTables.SetPersistence(autoPublisher.getTopic(), true);
//        String autoProfile = autoSubscriber.get();
//        var autoCommand = autoSelector.select(autoProfile);
//        scheduler.scheduleAutoCommand(autoCommand);
//    }

    @Override
    public void teleopSequence() {

    }

    @Override
    public void testSequence() {

    }
}
