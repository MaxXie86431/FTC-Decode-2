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
    private static final Pose startPose = new Pose(25, 127, Math.toRadians(135));
    private static final Pose launchPose = new Pose(50, 85, Math.toRadians(125));
    private static final Pose topRowEndPose = new Pose(15, 85, Math.toRadians(180));
    private static final Pose middleRowStartPose = new Pose(50, 60, Math.toRadians(180));
    private static final Pose middleRowEndPose = new Pose(15, 60, Math.toRadians(180));
    private static final Pose bottomRowStartPose = new Pose(50, 35, Math.toRadians(180));
    private static final Pose bottomRowEndPose = new Pose(15, 35, Math.toRadians(180));

    private PathChain initialLaunchPath;
    private PathChain topRowPath;
    private PathChain middleRowPath;

    private PathChain bottomRowPath;

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    //open (0.2) is logo on left closed (0) is logo on right
    private Command moveServo = new SetPosition(servo, 0.2).requires(this);
    private Command autonomousRoutine(){
        return new SequentialGroup(
            new FollowPath(initialLaunchPath),
            new FollowPath(topRowPath),
            new FollowPath(middleRowPath)
        );
    }
    public void buildPaths() {
        initialLaunchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        topRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, topRowEndPose))
                .addPath(new BezierLine(topRowEndPose, launchPose))
                .addPath(new BezierLine(launchPose, middleRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), topRowEndPose.getHeading())
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .addPath(new BezierLine(middleRowEndPose, launchPose))
                .setLinearHeadingInterpolation(middleRowStartPose.getHeading(), middleRowEndPose.getHeading())
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addPath(new BezierLine(bottomRowEndPose, launchPose))
                .setLinearHeadingInterpolation(bottomRowStartPose.getHeading(), bottomRowEndPose.getHeading())
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
