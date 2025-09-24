package org.firstinspires.ftc.teamcode.autonomous;
//package org.firstinspires.ftc.teamcode.guide.java.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Claw;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

@Autonomous (name = "Test auto for NextFTC >:|-] )")
public class NextFTCServo extends NextFTCOpMode {
    private ServoEx servo = new ServoEx("Servo");

    //Adding pedropathing component
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    //go forward :D
    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose endPose = new Pose(72, 82, Math.toRadians(0));

    //turn servo????????????????????????????????????????????????????????????????????????????????
    public Command moveServo = new SetPosition(servo, 0.2).requires(this);

    Path path = new Path(new BezierLine(startPose, endPose));
    private Command autoPath() {
        return new SequentialGroup(
                //new FollowPath(path),
                //new Delay(0.5),
                moveServo       //I have no clue if this works 💀
        );
    }

    @Override
    public void onStartButtonPressed() {
        autoPath().schedule();
    }

}