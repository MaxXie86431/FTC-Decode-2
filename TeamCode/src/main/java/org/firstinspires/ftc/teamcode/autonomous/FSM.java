package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;

import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Multiple Paths Please Work TSPMO")
@Configurable
public class FSM extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain firstPath, secondPath;


    private static Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private static Pose scorePose = new Pose(72, 100, Math.toRadians(90));
    private static Pose pickPose = new Pose(100,80,Math.toRadians(-36));

    public void buildPaths() {
        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        secondPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(firstPath, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(secondPath, true);
                    setPathState(2);
                }
                break;
        }
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();


    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        //drawCurrentAndHistory(); // optional visualization
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.update();
    }


}
