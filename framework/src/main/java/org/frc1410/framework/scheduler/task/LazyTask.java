package org.frc1410.framework.scheduler.task;


import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.framework.scheduler.task.impl.CommandTask;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.Set;
import java.util.function.Supplier;

public final class LazyTask implements Task {

	private final @NotNull Supplier<Task> supplier;
	private Task wrapped;

	public LazyTask(@NotNull Supplier<Task> supplier) {
		this.supplier = supplier;
	}

	@Contract("_ -> new")
	public static LazyTask fromCommand(@NotNull Supplier<Command> commandSupplier) {
		return new LazyTask(() -> new CommandTask(commandSupplier.get()));
	}

	@Override
	public void init() {
		if (wrapped == null) {
			wrapped = supplier.get();
		}

		wrapped.init();
	}

	@Override
	public void execute() {
		if (wrapped == null) return;
		wrapped.execute();
	}

	@Override
	public boolean isFinished() {
		return wrapped == null || wrapped.isFinished();
	}

	@Override
	public void end(boolean interrupted) {
		if (wrapped == null) return;

		wrapped.end(interrupted);
		wrapped = null;
	}

	@Override
	public @NotNull Set<? extends @NotNull Object> getLockKeys() {
		if (wrapped == null) return Set.of();
		return wrapped.getLockKeys();
	}
}