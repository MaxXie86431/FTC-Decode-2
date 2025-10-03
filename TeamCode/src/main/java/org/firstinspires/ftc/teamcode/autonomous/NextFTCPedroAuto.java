package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.hardware.positionable.SetPosition;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.ServoEx;

@Autonomous(name = "NextFTC Pedro Auto")
public class NextFTCPedroAuto extends NextFTCOpMode {
    private ServoEx servo = new ServoEx("Servo");
    // Define poses
    private static final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private static final Pose scorePose = new Pose(100, 100, Math.toRadians(0));
    private static final Pose endPose = new Pose(30, 100);
    private PathChain scorePreload;
    private PathChain newPath;

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    //open (0.2) is logo on left closed (0) is logo on right
    private Command moveServo = new SetPosition(servo, 0.2).requires(this);
    private Command autonomousRoutine(){
        return new ParallelGroup(
                new SequentialGroup(
                    new FollowPath(scorePreload),
                    new FollowPath(newPath)
                ),
                moveServo
        );
    }
    public void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        newPath = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        // Initialize the follower with your constants
        follower().setStartingPose(startPose);
        buildPaths();
    }


    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

}
