package org.frc1410.framework.util.log;

import org.frc1410.framework.util.log.transport.Transport;
import org.frc1410.framework.util.log.transport.Transports;
import org.intellij.lang.annotations.PrintFormat;
import org.jetbrains.annotations.NotNull;

import java.util.*;

public final class Logger {

	private final String name;
	private Set<Transport> transports = Set.of(Transports.CONSOLE);

	public Logger(String name) {
		this.name = name;
	}

	public void transports(@NotNull Transport @NotNull... transports) {
		this.transports = Set.of(transports);
	}

	public String getName() {
		return name;
	}

	public void log(@NotNull Log log) {
		transports.forEach(target -> target.accept(log));
	}

	public void log(@NotNull LogLevel level, @NotNull String message) {
		log(new Log(this, level, message));
	}

	public void log(@NotNull LogLevel level, @NotNull String format, @NotNull Object... args) {
		log(new Log(this, level, String.format(format, args)));
	}

	public void debug(@NotNull String message) {
		log(LogLevel.DEBUG, message);
	}

	public void info(@NotNull String message) {
		log(LogLevel.INFO, message);
	}

	public void warn(@NotNull String message) {
		log(LogLevel.WARN, message);
	}

	public void error(@NotNull String message) {
		log(LogLevel.ERROR, message);
	}

	public void fatal(@NotNull String message) {
		log(LogLevel.FATAL, message);
	}

	public void debug(@NotNull @PrintFormat String format, @NotNull Object... args) {
		log(LogLevel.DEBUG, format, args);
	}

	public void info(@NotNull @PrintFormat String format, @NotNull Object... args) {
		log(LogLevel.INFO, format, args);
	}

	public void warn(@NotNull @PrintFormat String format, @NotNull Object... args) {
		log(LogLevel.WARN, format, args);
	}

	public void error(@NotNull @PrintFormat String format, @NotNull Object... args) {
		log(LogLevel.ERROR, format, args);
	}

	public void fatal(@NotNull @PrintFormat String format, @NotNull Object... args) {
		log(LogLevel.FATAL, format, args);
	}
}