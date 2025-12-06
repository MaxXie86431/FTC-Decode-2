package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Flywheel implements Subsystem{
    public static double kP = 0.005;
    public static double kI = 0.01;
    public static double kD = 0;
    public static double kV = 0.01;
    public static double kA = 0.02;
    public static double kS = 0.03;
    private final ControlSystem controller = ControlSystem.builder()
            .velPid(kP, kI, kD)
            .basicFF(kV, kA, kS)
            .build();
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }
    private final MotorEx motor = new MotorEx("BottomLaunch");
    private static final double TICKS_PER_REVOLUTION = 2240.0;
    public static int outVelocity = 1000;
    public static int inVelocity = -200;

    public double getVelocityRPM(){
        double ticksPerSecond = motor.getVelocity();
        /*double revPerSec = ticksPerSecond / TICKS_PER_REVOLUTION;
        double degreesPerSecond = revPerSec * 360;*/
        return ticksPerSecond;
    }

    public Command shootOut(double velocity) {
        //double ticksPerSecond = velocity * TICKS_PER_REVOLUTION / 60.0;
        return new RunToVelocity(controller, velocity).requires(this);
    }

    public Command off() {
        return new RunToVelocity(controller,0).requires(this);
    }
    public Command reverse() {
        return new RunToVelocity(controller,inVelocity).requires(this);
    }

    public Command inward() {
        return new ParallelGroup(
                Intermediate.INSTANCE.rolldown(),
                new SetPower(motor, -0.2)
        ).requires(this);
    }
    public Command outward(double power, double delay) {
        return new SequentialGroup(
                Intermediate.INSTANCE.rollup(),
                new Delay(delay),
                new RunToVelocity(controller, 500)
        ).requires(this);
    }
    public Command stop(){
        return new ParallelGroup(
                new SetPower(motor, 0),
                Intermediate.INSTANCE.stop()
        ).requires(this);
    }

    @Override
    public void periodic() {
        motor.setPower(controller.calculate(motor.getState()));
    }
}
