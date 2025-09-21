package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="TS")
@Configurable
public class TestAuto extends OpMode {
    private Follower follower;
    private PathChain firstPath;

    private static Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private static Pose scorePose = new Pose(72, 100, Math.toRadians(90));
    private static Pose pickPose = new Pose(100,80,Math.toRadians(-36));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    @Override
    public void start() {
        follower.followPath(firstPath);
    }

    @Override
    public void loop() {
        follower.update();
        //drawCurrentAndHistory(); // optional visualization
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.update();
    }
}
