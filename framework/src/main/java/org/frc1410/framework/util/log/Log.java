package org.frc1410.framework.util.log;

public class Log {

	public final Logger source;
	public final LogLevel level;
	public final String message; // TODO • support complex logs. This is complex, and is blocked by NT stuff.

	public Log(Logger source, LogLevel level, String message) {
		this.source = source;
		this.level = level;
		this.message = message;
	}
}