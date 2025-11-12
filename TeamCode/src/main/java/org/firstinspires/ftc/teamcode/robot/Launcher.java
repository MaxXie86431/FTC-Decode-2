package org.firstinspires.ftc.teamcode.robot;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.time.Duration;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Launcher implements Subsystem {

    public static final Launcher INSTANCE = new Launcher();
    private Launcher() {}
    private MotorEx motor1;
    private MotorEx motor2;
    private MotorEx motor3;

    public void init() {
        motor1 = new MotorEx("BottomLaunch");
        motor2 = new MotorEx("TopLaunch");
    }
    public Command inwards() {
        return new SequentialGroup(
                new ParallelGroup(
                        new ParallelGroup(
                                new SetPower(motor1, 0.2),
                                new SetPower(motor2, 0.2)
                        )
                )
        ).requires(this);
    }
    public Command outward(double power, double delay) {
        return new SequentialGroup(
            new ParallelGroup(
                new ParallelGroup(
                        new SetPower(motor1, power),
                        new SetPower(motor2, power)
                ),
                new SequentialGroup(
                    new Delay(delay),
                    Intermediate.INSTANCE.rollup()
                )
            )
        ).requires(this);
    }
    public Command stop(){
        return new ParallelGroup(
                new ParallelGroup(
                        new SetPower(motor1, 0),
                        new SetPower(motor2, 0)
                ),
                Intermediate.INSTANCE.stop()
        ).requires(this);
    }
}