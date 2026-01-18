package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;

// Limelight dependencies
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Limelight Color Test")
@Configurable
public class LimelightTest extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    private PathChain rotationPath;
    private String lastDetectedLeftmostColor = "";
    private static final Pose startPose = new Pose(72, 72, Math.toRadians(90));

    private String classNameToColor(String className) {
        String lower = className.toLowerCase();
        if (lower.contains("green")) {
            return "GREEN";
        } else if (lower.contains("purple")) {
            return "PURPLE";
        }
        return "unknown";
    }

    // returns List["GREEN", "PURPLE", ...] sorted left to right
    public List<String> getDetectedBallsLeftToRight() {
        List<String> sortedColors = new ArrayList<>();

        LLResult result = limelight.getLatestResult();
        //telemetry.addData("Data_result", result);
        //telemetry.update();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            //telemetry.addData("Data", detectorResults);
            //telemetry.update();
            if (detectorResults != null && !detectorResults.isEmpty()) {
                // Filter valid detections
                List<LLResultTypes.DetectorResult> validDetections = new ArrayList<>();
                for (LLResultTypes.DetectorResult detection : detectorResults) {
                    String color = classNameToColor(detection.getClassName());
                    if (!color.equals("unknown") && detection.getConfidence() > 0.5) {
                        validDetections.add(detection);
                        telemetry.addData("Color", color);

                    }
                }

                // Sort by X pixels (left to right)
                Collections.sort(validDetections, new Comparator<LLResultTypes.DetectorResult>() {
                    @Override
                    public int compare(LLResultTypes.DetectorResult a, LLResultTypes.DetectorResult b) {
                        return Double.compare(a.getTargetXPixels(), b.getTargetXPixels());
                    }
                });

                // Extract colors in sorted order
                for (LLResultTypes.DetectorResult detection : validDetections) {
                    sortedColors.add(classNameToColor(detection.getClassName()));
                }
            }
        }

        telemetry.addData("Detected Colors L->R", sortedColors.toString());
        telemetry.addData("Leftmost Color", sortedColors.isEmpty() ? "None" : sortedColors.get(0));
        telemetry.update();

        return sortedColors;
    }
    @Override
    public void loop() {
        follower.update();

        List<String> detectedColors = getDetectedBallsLeftToRight();

    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized - Limelight Color Detection Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
