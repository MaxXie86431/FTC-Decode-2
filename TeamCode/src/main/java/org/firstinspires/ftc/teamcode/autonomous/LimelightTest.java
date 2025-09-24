package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
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
    private boolean rotationExecuted = false;
    private PathChain rotationPath;
    private String lastDetectedLeftmostColor = "";
    private static final Pose startPose = new Pose(72, 72, Math.toRadians(90));

    private String classNameToColor(String className) {
        switch (className.toLowerCase()) {
            case "blue_block":
                return "BLUE";
            case "red_block":
                return "RED";
            case "yellow_block":
                return "YELLOW";
            default:
                return "unknown";
        }
    }

    // returns List["BLUE", "RED", "YELLOW", ...] sorted left to right
    public List<String> getDetectedBallsLeftToRight() {
        List<String> sortedColors = new ArrayList<>();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            if (detectorResults != null && !detectorResults.isEmpty()) {
                // Filter valid detections
                List<LLResultTypes.DetectorResult> validDetections = new ArrayList<>();
                for (LLResultTypes.DetectorResult detection : detectorResults) {
                    String color = classNameToColor(detection.getClassName());
                    if (!color.equals("unknown") && detection.getConfidence() > 0.5) {
                        validDetections.add(detection);
                    }
                }

                // Sort by x-coordinate (left to right)
                Collections.sort(validDetections, new Comparator<LLResultTypes.DetectorResult>() {
                    @Override
                    public int compare(LLResultTypes.DetectorResult a, LLResultTypes.DetectorResult b) {
                        return Double.compare(a.getTargetXDegrees(), b.getTargetXDegrees());
                    }
                });

                // Extract colors in sorted order
                for (LLResultTypes.DetectorResult detection : validDetections) {
                    sortedColors.add(classNameToColor(detection.getClassName()));
                }
            }
        }

        return sortedColors;
    }

    private String getLeftmostColor() {
        List<String> colors = getDetectedBallsLeftToRight();
        if (!colors.isEmpty()) {
            return colors.get(0); // Return leftmost (first) color
        }
        return "";
    }

    private void buildRotationPath(double rotationDegrees) {
        Pose currentPose = follower.getPose();
        double rotationRadians = Math.toRadians(rotationDegrees);
        double newHeading = currentPose.getHeading() + rotationRadians;

        Pose rotationPose = new Pose(currentPose.getX(), currentPose.getY(), newHeading);

        rotationPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, rotationPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), newHeading)
                .build();
    }

    @Override
    public void loop() {
        follower.update();

        List<String> detectedColors = getDetectedBallsLeftToRight();
        String leftmostColor = getLeftmostColor();

        if (!rotationExecuted && !leftmostColor.isEmpty()) {
            rotationExecuted = true;
            lastDetectedLeftmostColor = leftmostColor;

            double rotationDegrees = 0;
            switch (leftmostColor) {
                case "RED":
                    rotationDegrees = 180;
                    break;
                case "BLUE":
                    rotationDegrees = 360;
                    break;
                case "YELLOW":
                    rotationDegrees = 540;
                    break;
            }

            if (rotationDegrees > 0) {
                buildRotationPath(rotationDegrees);
                follower.followPath(rotationPath, true);
                telemetry.addData("Action", "Rotating %.0f degrees - %s detected leftmost!", rotationDegrees, leftmostColor);
            }
        } else if (rotationExecuted && !follower.isBusy()) {
            telemetry.addData("Action", "Rotation completed for %s", lastDetectedLeftmostColor);
        }

        telemetry.addData("Detected Colors L->R", detectedColors.toString());
        telemetry.addData("Leftmost Color", leftmostColor.isEmpty() ? "None" : leftmostColor);
        telemetry.addData("Rotation Executed", rotationExecuted);

        // Show detailed detection information
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            if (detections != null && !detections.isEmpty()) {
                telemetry.addData("Total Detections", detections.size());
                for (int i = 0; i < detections.size() && i < 5; i++) {
                    LLResultTypes.DetectorResult detection = detections.get(i);
                    String className = detection.getClassName();
                    double x = detection.getTargetXDegrees();
                    double y = detection.getTargetYDegrees();
                    double confidence = detection.getConfidence();
                    telemetry.addData(className, String.format("at (%.1f, %.1f)° conf:%.2f", x, y, confidence));
                }
            } else {
                telemetry.addData("Status", "No objects detected");
            }
        }

        telemetry.update();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized - Limelight Color Detection Ready");
        telemetry.addData("Instructions", "RED=180°, BLUE=360°, YELLOW=540°");
        telemetry.addData("Note", "Robot will only rotate, never move");
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
