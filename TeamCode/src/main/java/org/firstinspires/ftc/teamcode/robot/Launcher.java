package org.firstinspires.ftc.teamcode.robot;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Launcher implements Subsystem {

    public static final Launcher INSTANCE = new Launcher();
    private Launcher() {}

   private MotorEx motor1;
   private MotorEx motor2;
    public void init() {
        motor1 = new MotorEx("BottomLaunch");
        motor2 = new MotorEx("TopLaunch");
    }
    public Command inwards() {
        return new ParallelGroup(
                new SetPower(motor1, 1),
                new SetPower(motor2, 1)
        );
    }
    public Command outward() {
        return new ParallelGroup(
                new SetPower(motor1, -1),
                new SetPower(motor2, -1)
        );
    }
    public Command stop(){
        return new ParallelGroup(
                new SetPower(motor1, 0),
                new SetPower(motor2, 0)
        );
    }
}