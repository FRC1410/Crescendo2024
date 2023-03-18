package org.frc1410.framework.util.log;

public enum LogLevel {

	DEBUG("\u001b[37m"),
	INFO("\u001b[37;1m"),
	WARN("\u001b[33;1m"),
	ERROR("\u001b[31m"),
	FATAL("\u001b[31;1m");

	private final String ansiColor;

	LogLevel(String ansiColor) {
		this.ansiColor = ansiColor;
	}

	public String getAnsiColor() {
		return ansiColor;
	}
}