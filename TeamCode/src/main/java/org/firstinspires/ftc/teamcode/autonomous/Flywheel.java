package org.firstinspires.ftc.teamcode.autonomous;

import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.powerable.SetPower;

public class Flywheel implements Subsystem{
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }
    private final MotorEx motor = new MotorEx("flywheel_motor");

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();
    public Command shootOut() {
        return new RunToVelocity(controller, 500).requires(this);
    }

    public Command off() {
        return new RunToVelocity(controller,0).requires(this);
    }
    public Command reverse() {
        return new RunToVelocity(controller,-100).requires(this);
    }
    @Override
    public void periodic() {
        motor.setPower(controller.calculate(motor.getState()));
    }
}
