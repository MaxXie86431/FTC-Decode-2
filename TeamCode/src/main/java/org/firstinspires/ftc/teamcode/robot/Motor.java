package org.firstinspires.ftc.teamcode.robot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Motor implements Subsystem {
    public static final Motor INSTANCE = new Motor();

    private Motor(){}

    MotorEx motor1 = new MotorEx("BottomLaunch")/*.reversed()*/;

    public Command launchOut =  new SetPower( motor1,1).requires(this);
    public Command launchIn = new SetPower(motor1, -1).requires(this);
    public Command stop =  new SetPower( motor1,0).requires(this);

}