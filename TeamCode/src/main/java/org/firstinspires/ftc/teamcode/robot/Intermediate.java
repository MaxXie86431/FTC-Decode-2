package org.firstinspires.ftc.teamcode.robot;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intermediate implements Subsystem{
    public static final Intermediate INSTANCE = new Intermediate();
    private Intermediate() {}
    private MotorEx roller;
    private MotorEx verticalroller;
    @Override
    public void initialize() {
        roller = new MotorEx("IntermediateMotor");
        verticalroller = new MotorEx("VerticalIntermediateMotor");
    }
    public Command rollup() {
        return new ParallelGroup(
                new SetPower(roller,1),
                new SetPower(verticalroller,1)
        ).requires(this);
    }
    public Command rolldown() {
        return new ParallelGroup(
                new SetPower(roller,-1),
                new SetPower(verticalroller,-1)
        ).requires(this);
    }

    public Command stop(){
        return new ParallelGroup(
                new SetPower(roller,0),
                new SetPower(verticalroller,0)
        ).requires(this);
    }

    public Command horizontalstop(){
        return new SetPower(roller,0);
    }
    public Command horizontalrollup(){
        return new SetPower(roller,1);
    }
    public Command horizontalrolldown(){
        return new SetPower(roller,-1);
    }
}
