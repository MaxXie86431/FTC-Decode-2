package org.firstinspires.ftc.teamcode.motorwrappers;

import com.sun.tools.doclint.Checker;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
import org.firstinspires.ftc.teamcode.motorwrappers.FlagConfig;

public class teleopMotorSubsystem implements Subsystem {
    public static final teleopMotorSubsystem INSTANCE = new teleopMotorSubsystem();
    private teleopMotorSubsystem() {}
    private MotorEx motor;
    public FlagConfig teleopflags = new FlagConfig();
    private Command inwardsCommand;
    private Command outwardsCommand;
    private Command stopCommand;
    @Override
    public void initialize() {
        motor = new MotorEx("BottomLaunch");
    }

    public Command inwards() {
        if (inwardsCommand == null) {
            inwardsCommand = new SetPower(motor, 0.2).requires(this);
        }
        return inwardsCommand;
    }

    public Command outwards(double power) {
        if (outwardsCommand == null) {
            outwardsCommand = new SetPower(motor, power).requires(this);
        }
        return outwardsCommand;
    }

    public Command stop() {
        if (stopCommand == null) {
            stopCommand = new SetPower(motor, 0).requires(this);
        }
        return stopCommand;
    }
    /*
    public static void buttonMotor(MotorEx motor, double power, double reversePower, boolean button1, boolean button2) {
        Command forward = new SetPower(motor,power);
        Command backward = new SetPower(motor,reversePower);
        Command stop = new SetPower(motor, 0);
        if(button1) {
            forward.schedule();
        }
        else if (button2) {
            backward.schedule();
        }
        else {
            stop.schedule();
        }
    }
     */

    @Override
    public void periodic() {
        if (teleopflags.launchOutFlag) {
            outwards(-1).schedule();
            teleopflags.launchOutFlag = false;
        }
        if (teleopflags.launchInFlag) {
            inwards().schedule();
            teleopflags.launchInFlag=false;
        }
        if (teleopflags.stopLaunchFlag) {
            stop().schedule();
            teleopflags.stopLaunchFlag = false;
        }

    }
}
