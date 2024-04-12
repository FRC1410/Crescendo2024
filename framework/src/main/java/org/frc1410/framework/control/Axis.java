package org.frc1410.framework.control;

import java.util.function.Function;

public class Axis {

	private final Controller controller;
	private final int id;
	private final AxisButton button;

	public Axis(Controller controller, int id) {
		this.controller = controller;
		this.id = id;
		this.button = new AxisButton(controller, this);
	}

	public double getRaw() {
		return controller.backingController.getRawAxis(id);
	}

	public double get() {
		double raw = getRaw();
		double mag = Math.abs(raw);

		if (mag <= controller.deadzone) {
			return 0;
		}

		return ((mag - controller.deadzone) / (1 - controller.deadzone)) * (raw / mag);
	}

	public AxisButton button() {
		return button;
	}

	public Axis transformed(Function<Double, Double> transformation) {
		return new TransformedAxis(this.controller, this.id, transformation);
	}

	public Axis negated() {
		return this.transformed((value) -> -value);
	}
}