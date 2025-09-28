package org.firstinspires.ftc.teamcode.robot;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Claw implements Subsystem {

    public static final Claw INSTANCE = new Claw();
    private Claw() {}

    private ServoEx servo = new ServoEx("Servo");

   public Command closeServo = new SetPosition(servo, 0).requires(this);
    public Command openServo = new SetPosition(servo, 0.1).requires(this);
    public Command moventurn() {
        final Pose firstPose = follower().getPose();
        final Pose endPose = new Pose(firstPose.getX(), firstPose.getY()+50, firstPose.getHeading());
        PathChain newPath = follower().pathBuilder()
                .addPath(new BezierLine(firstPose, endPose))
                .setLinearHeadingInterpolation(firstPose.getHeading(), endPose.getHeading())
                .build();
        return new SequentialGroup(
                new FollowPath(newPath),
                openServo
        );
    }

}