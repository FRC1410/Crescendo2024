package org.frc1410.chargedup2023.Commands.Auto;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance extends CommandBase {
    private final Drivetrain drivetrain;
    private final PIDController pidController;

    public Balance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.pidController = new PIDController(0.04, 0, 0.001);
        this.pidController.setTolerance(6, 2);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.pidController.setSetpoint(0);
    }

    @Override
    public void execute() {
        var currentAngle = drivetrain.getPitch().getDegrees();
        // System.out.println("exec " + currentAngle);

        if (currentAngle > 6 || currentAngle < -6) {
			var controllerOutput = pidController.calculate(currentAngle);

            // System.out.println("output " + controllerOutput);

			// If somebody changes this they will be skinned alive
            // var power = -(Math.log1p(Math.min(controllerOutput, 1)))/Math.log1p(4);

            drivetrain.drive(controllerOutput, 0, 0, false);
		} else {
			drivetrain.drive(0, 0, 0, false);
		}
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
    }
}
