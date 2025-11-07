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

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    private MotorEx intakeMotor;
    public void init() {
        intakeMotor = new MotorEx("Intake-Motor");
    }
    public Command inwards() {
        return new ParallelGroup(
                new SetPower(intakeMotor, 1),
                Intermediate.INSTANCE.rolldown()

        );
    }

    public Command outward() {
        return new ParallelGroup(
            new SetPower(intakeMotor, -1),
            Intermediate.INSTANCE.rollup()
        );
    }

    public Command stop() {
        return new ParallelGroup(
                new SetPower(intakeMotor, 0),
                Intermediate.INSTANCE.stop()
        );
    }
    /*
    public Command inwards() {
        return new SetPower(intakeMotor, 1);
    }
    public Command stop() {
        return new SetPower(intakeMotor, 0);
    }
    public Command outward(){
        return new SetPower(intakeMotor, -1);
    }

     */
}