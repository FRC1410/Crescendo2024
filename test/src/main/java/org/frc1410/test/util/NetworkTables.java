package org.frc1410.test.util;

import edu.wpi.first.networktables.*;

public interface NetworkTables {
	
	/**
	 * Creates a publisher for a topic in a given table with a starting value
	 * @param table NetworkTable, desired table for Publisher
	 * @param name String, desired name for topic
	 * @param startingValue double, desired starting value
	 * @return DoublePublisher
	 */
	static DoublePublisher PublisherFactory(NetworkTable table, String name, double startingValue) {
		DoublePublisher publisher = table.getDoubleTopic(name).publish();
		publisher.set(startingValue);
		publisher.setDefault(startingValue);
		return publisher;
	}

	static DoublePublisher PublisherFactory(NetworkTable table, String name, double startingValue, double defaultValue) {
		DoublePublisher publisher = table.getDoubleTopic(name).publish();
		publisher.set(startingValue);
		publisher.setDefault(defaultValue);
		return publisher;
	}

	/**
	 * Creates a subscriber for a topic from a table
	 * @param table Networktable, desired table for subscriber
	 * @param topic DoubleTopic, desired topic for subscriber
	 * @return DoubleSubscriber
	 */
	static DoubleSubscriber SubscriberFactory(NetworkTable table, DoubleTopic topic) {
		String name = topic.getName().substring(topic.getName().lastIndexOf("/") + 1);
		return table.getDoubleTopic(name).subscribe(0.0);
	}

	static DoubleSubscriber SubscriberFactory(NetworkTable table, DoubleTopic topic, double defaultValue) {
		String name = topic.getName().substring(topic.getName().lastIndexOf("/") + 1);
		return table.getDoubleTopic(name).subscribe(defaultValue);
	}

	static StringPublisher PublisherFactory(NetworkTable table, String name, String startingValue) {
		StringPublisher publisher = table.getStringTopic(name).publish();
		publisher.set(startingValue);
		publisher.setDefault(startingValue);
		return publisher;
	}

	static StringSubscriber SubscriberFactory(NetworkTable table, StringTopic topic) {
		String name = topic.getName().substring(topic.getName().lastIndexOf("/") + 1);
		return table.getStringTopic(name).subscribe("");
	}

	static BooleanPublisher PublisherFactory(NetworkTable table, String name, boolean startingValue) {
		BooleanPublisher publisher = table.getBooleanTopic(name).publish();
		publisher.set(startingValue);
		publisher.setDefault(startingValue);
		return publisher;
	}

	static BooleanSubscriber SubscriberFactory(NetworkTable table, BooleanTopic topic) {
		String name = topic.getName().substring(topic.getName().lastIndexOf("/") + 1);
		return table.getBooleanTopic(name).subscribe(false);
	}

	static void SetPersistence(Topic topic, boolean persistent) {
		topic.setPersistent(persistent);
	}

	static boolean GetPersistence(Topic topic) {
		return topic.isPersistent();
	}

	/* Setup for any class requiring network tables
	NetworkTableInstance instance = NetworkTableInstance.getDefault();
	NetworkTable table = instance.getTable("Test");

	DoublePublisher pub = Networktables.PublisherFactory(table, "Testing", 0);
	DoubleSubscriber sub = Networktables.SubscriberFactory(table, pub.getTopic(), 0);
	 */
}