package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Intermediate;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Limelight;

import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "From Starting Position -> Intake Balls -> Launch Balls -> Back to initial position")
public class AutoMachine extends NextFTCOpMode {
    // Define poses
    private static final Pose path1 = new Pose(56, 8, Math.toRadians(90));
    private static final Pose path2 = new Pose(56,36, Math.toRadians(180));
    private static final Pose path3 = new Pose(35,36, Math.toRadians(180));
    private static final Pose path4 = new Pose(35,36, Math.toRadians(100));
    public static int velocity = 1000;
    private PathChain p1, p2, p3, p4;


    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Intermediate.INSTANCE, Flywheel.INSTANCE, Limelight.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }


    @Override
    public void onInit() {
        follower().setStartingPose(startPose);
        p1 = follower().pathBuilder()
                .addPath(new BezierLine(path1, path2))
                .setLinearHeadingInterpolation()
                .addPath(new BezierLine(path2, path3))
                .setTangentHeadingInterpolation()
                .build();
        p2 = follower().pathBuilder()
                .addPath(new BezierLine(path3, path4))
                .setLinearHeadingInterpolation()
                .build();

    }


    private Command executePaths(){
        return new SequentialGroup(
                new FollowPath(p1),
                Intake.INSTANCE.inward(),
                new FollowPath(p2),
                Limelight.INSTANCE.alignToTarget(),
                Flywheel.INSTANCE.constantShot(velocity)
        );
    }

    @Override
    public void onStartButtonPressed() {
        executePaths().schedule();
    }

}
