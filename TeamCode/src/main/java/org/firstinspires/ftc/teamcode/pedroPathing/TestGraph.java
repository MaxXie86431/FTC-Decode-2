package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Graph")
public class TestGraph extends OpMode {
    //private final GraphManager graphManager = PanelsGraph.INSTANCE.getManager();
    static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    
    private Follower follower;

    private final ElapsedTime timer = new ElapsedTime();

    private double sinVariable = 0.0;
    private double cosVariable = 0.0;
    private double constVariable = 0.0;
    private double errorVariable = 0.0;

    @Override
    public void init() {
        // Initialize follower for this OpMode
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        
        timer.reset();
        updateSignals();
    }

    @Override
    public void loop() {
        follower.update();
        updateSignals();
    }

    private void updateSignals() {
        double t = timer.seconds();
        sinVariable = Math.sin(t);
        cosVariable = Math.cos(t);
        errorVariable = follower.getHeadingError();

        panelsTelemetry.addData("sin", sinVariable);
        panelsTelemetry.addData("cos", cosVariable);
        panelsTelemetry.addData("const", constVariable);
        panelsTelemetry.addData("error", follower.getHeadingError());
        panelsTelemetry.addData("goal", 0);
        panelsTelemetry.update(telemetry);
    }
}

