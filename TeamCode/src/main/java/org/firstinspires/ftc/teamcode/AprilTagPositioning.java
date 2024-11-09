package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
        aprilTagFieldPositions.put(11, new VectorF(-72.0f, 48.0f, 5.9f));  // Example position for tag 1
        aprilTagFieldPositions.put(12, new VectorF(0.0f, 72.0f, 5.9f));  // Example position for tag 2
        aprilTagFieldPositions.put(13, new VectorF(72.0f, 48.0f, 5.9f)); // Example position for tag 3
        aprilTagFieldPositions.put(14, new VectorF(72.0f, -48.0f, 5.9f));  // Example position for tag 1
        aprilTagFieldPositions.put(15, new VectorF(0.0f, -72.0f, 5.9f));  // Example position for tag 2
        aprilTagFieldPositions.put(16, new VectorF(-72.0f, -48.0f, 5.9f)); // Example position for tag 3
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
    }

    private void estimateRobotPosition() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder positions = new StringBuilder();

        for (AprilTagDetection detection : currentDetections) {
            int id = detection.id;

            // Check if the detected tag ID has a known position
            if (aprilTagFieldPositions.containsKey(id)) {
                VectorF fieldPosition = aprilTagFieldPositions.get(id);
                 // Pose translation relative to camera


                    // Tag position relative to the camera's origin
                    double tagCameraX = detection.ftcPose.y;
                    double tagCameraY = detection.ftcPose.x;
                    double tagCameraZ = detection.ftcPose.z;
                telemetry.addData("filed position x", fieldPosition.get(0));
                telemetry.addData("field position y", fieldPosition.get(1));
                telemetry.addData("field position z", fieldPosition.get(2));
                telemetry.addData("ftcposition x", tagCameraX);
                telemetry.addData("ftcposition y", tagCameraY);




                telemetry.update();
                    // Estimate the robot's position based on the tag's position and camera offset
                    double estimatedRobotX = Math.round(fieldPosition.get(0) - tagCameraX - cameraOffsetX);
                    double estimatedRobotY = Math.round(fieldPosition.get(1) - tagCameraY - cameraOffsetY);
                    double estimatedRobotZ = Math.round(fieldPosition.get(2) - tagCameraZ - cameraOffsetZ);
                telemetry.addData("Estimate position x", estimatedRobotX);
                telemetry.addData("Estimate position y", estimatedRobotY);
                telemetry.update();
                    positions.append("Tag ID: ").append(id)
                            .append(" | Robot X: ").append(estimatedRobotX)
                            .append(" | Robot Y: ").append(estimatedRobotY)
                            .append(" | Robot Z: ").append(estimatedRobotZ)
                            .append("\n");
                }


        }






    }
}
