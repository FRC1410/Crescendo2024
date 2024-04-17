package org.frc1410.framework.control;

import java.util.function.Function;

public class TransformedAxis extends Axis {
    private final Function<Double, Double> transformation;

    public TransformedAxis(Controller controller, int id, Function<Double, Double> transformation) {
        super(controller, id);
        this.transformation = transformation;
    }
    
    @Override
    public double getRaw() {
        return this.transformation.apply(super.getRaw());
    }

    @Override
    public double get() {
        return this.transformation.apply(super.get());
    }
}
