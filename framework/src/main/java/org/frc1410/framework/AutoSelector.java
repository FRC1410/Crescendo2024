package org.frc1410.framework;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.frc1410.framework.util.log.Logger;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Unmodifiable;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * A utility class for storing auto profiles and getting them from a profile
 * name. Auto profile commands are lazily loaded, and are only generated for
 * when they are needed. The list of profiles can also be accessed.
 */
public final class AutoSelector {

	private static final Logger LOG = new Logger("AutoSelector");

	private final List<@NotNull AutoProfile> profiles = new ArrayList<>();

	/**
	 * Adds a profile to this auto selector.
	 *
	 * @param name The name of the profile (generally exposed in dashboards).
	 * @param commandSupplier A {@link Supplier} for the auto command that is
	 *						invoked when the profile is selected.
	 *
	 * @return This {@link AutoSelector} for chaining.
	 * @throws NullPointerException If {@code name} or {@code commandSupplier} is null.
	 */
	@Contract("_, _ -> this")
	public @NotNull AutoSelector add(@NotNull String name, @NotNull Supplier<@NotNull Command> commandSupplier) {
		var profile = new AutoProfile(name, commandSupplier, profiles.size());
		profiles.add(profile);
		return this;
	}

	/**
	 * Selects an auto profile, building its associated {@link Command}.
	 *
	 * @param profileName The selected profile name.
	 *
	 * @return The {@link Command} to be scheduled for this profile.
	 * @throws NullPointerException If {@code profileName} is null.
	 */
	public @NotNull Command select(@NotNull String profileName) {
		for (var profile : profiles) {
			if (profileName.equalsIgnoreCase(profile.name())) {
				return Objects.requireNonNull(profile.supplier().get(), "Generated auto command must not be null");
			}
		}

		LOG.warn("Auto profile was invalid: \"" + profileName + "\" (skipping)");
		return new InstantCommand(() -> {});
	}

	/**
	 * Gets a list of the registered auto profiles in this selector.
	 *
	 * @return A never-null, unmodifiable {@link List} of all profiles.
	 */
	public @NotNull @Unmodifiable List<@NotNull AutoProfile> getProfiles() {
		return List.copyOf(profiles);
	}
}