package org.frc1410.framework;

import edu.wpi.first.wpilibj2.command.Command;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;
import java.util.function.Supplier;

public record AutoProfile(@NotNull String name, @NotNull Supplier<@NotNull Command> supplier, int id) {
	public AutoProfile {
		Objects.requireNonNull(name);
		Objects.requireNonNull(supplier);
	}
}