package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "NextFTC Pedro Auto")
public class NextFTCPedroAuto extends NextFTCOpMode {

    // Define poses
    private static final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private static final Pose scorePose = new Pose(100, 100, Math.toRadians(0));
    private PathChain scorePreload;

    {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private Command autonomousRoutine(){
        return new SequentialGroup(
                new FollowPath(scorePreload)
        );
    }
    public void buildPaths() {
        scorePreload = follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
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
