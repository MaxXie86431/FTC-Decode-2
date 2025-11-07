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

    public void init() {
        roller = new MotorEx("IntermediateMotor");
    }
    public Command rollup() {
        return new SetPower(roller,1);
    }
    public Command rolldown() {
        return new SetPower(roller,-1);
    }

    public Command stop(){
        return new SetPower(roller,0);
    }
}
