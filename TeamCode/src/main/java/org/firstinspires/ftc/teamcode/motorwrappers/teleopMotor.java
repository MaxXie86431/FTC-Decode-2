package org.firstinspires.ftc.teamcode.motorwrappers;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class teleopMotor {
    public static void buttonMotor(MotorEx motor, double power, double reversePower, Button button1, Button button2) {
        Command forward = new SetPower(motor,power);
        Command backward = new SetPower(motor,reversePower);
        Command stop = new SetPower(motor, 0);
        if(button1.get()) {
            forward.schedule();
        }
        else if (button2.get()) {
            backward.schedule();
        }
        else {
            stop.schedule();
        }
    }
}
