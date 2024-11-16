package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.HashMap;
import java.util.List;

@Autonomous
public class AprilTagPositioning extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Define the camera's position relative to the robot's center (in inches or cm)
    private final double cameraOffsetX = 3.0; // Example offset in X direction
    private final double cameraOffsetY = 0.0; // Example offset in Y direction
    private final double cameraOffsetZ = 6.0; // Example offset in Z direction

    // Map to hold the fixed field positions of the AprilTags based on their IDs
    private final HashMap<Integer, VectorF> aprilTagFieldPositions = new HashMap<>();

    @Override
    public void init() {
        // Define fixed positions for AprilTags on the field

        // Add other tag positions as needed...

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }

    @Override
    public void init_loop() {
        estimateRobotPosition();
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        // Autonomous actions based on position can go here
        estimateRobotPosition();
    }

    private void estimateRobotPosition() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder positions = new StringBuilder();
    telemetry.addLine("estimateRobotPosition lance") ;
            // Pose translation relative to camera
        for (AprilTagDetection detection : currentDetections) {
            int id = detection.id;

                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                                detection.robotPose.getPosition().x,
                                detection.robotPose.getPosition().y,
                                detection.robotPose.getPosition().z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                                detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                                 detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                                (detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)+90)));
                        telemetry.update();





                    // Estimate the robot's position based on the tag's position and camera offset




        }






    }
}}
