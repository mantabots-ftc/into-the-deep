

package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@TeleOp
public class MechanumDrive extends OpMode {

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    double botHeading ;
    IMU imu;
    boolean aIsPressed = false;
    double imuTarget;
    double replace;
    double difference;
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    private ColorSensor colorSensor;
    double distance1;
    double distance2;
    @Override
    public void init (){
       frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
       backLeft = hardwareMap.get(DcMotor.class,"backLeft");
       frontRight = hardwareMap.get(DcMotor.class,"frontRight");
       backRight = hardwareMap.get(DcMotor.class,"backRight");
       backRight.setDirection(DcMotorSimple.Direction.REVERSE);
       backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       distanceSensor1 = hardwareMap.get(DistanceSensor.class,"sensor_color_distance");
       distanceSensor2 = hardwareMap.get(DistanceSensor.class,"sensor_color_distance2");
       double multiplier;



         imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD ));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

         botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }
    @Override
    public void loop(){

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;
        distance1 = distanceSensor1.getDistance(DistanceUnit.CM) ;
        distance2 = distanceSensor2.getDistance(DistanceUnit.CM) ;
        telemetry.addData("distance 1:",distance1);
        telemetry.addData("distance 2:",distance2);
        telemetry.addData("ImuTarget", imuTarget);
        telemetry.addData("botHeading", botHeading);
        telemetry.addData("aIsPressed",aIsPressed);
        telemetry.addData("difference",difference);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double multiplier = 0.45;
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        difference = imuTarget-botHeading;
        while (difference < 0) { difference += 2 * Math.PI; }
        while (difference >= 2 * Math.PI) { difference -= 2 * Math.PI; }
        if (botHeading < 0){
            botHeading += Math.PI*2;
        }



        if (gamepad1.left_bumper) {
            multiplier = 0.9;
        }
        if(gamepad1.right_bumper){
            multiplier=0.25;
        }
        if(gamepad1.a){
            aIsPressed = true;

             imuTarget =  (botHeading + Math.PI)%(Math.PI*2);
             if (imuTarget > 2*Math.PI){
                 imuTarget -= Math.PI*2;
             }


        }



        if (aIsPressed){



            if (Math.abs(difference) >0.1) {

                telemetry.addData("ImuTarget", imuTarget);
                telemetry.addData("botHeading", botHeading);
                telemetry.addData("aIsPressed",aIsPressed);
                telemetry.addData("difference",difference);

                frontLeft.setPower(-0.6);
                backLeft.setPower(-0.6);
                frontRight.setPower(0.6);
                backRight.setPower(0.6);
                telemetry.update();
            }else aIsPressed = false;
        }


        frontLeft.setPower(((y+x+rotation)/denominator)* multiplier*1.1);
        backLeft.setPower(((y-x+rotation)/denominator)* multiplier*1.1);
        frontRight.setPower(((y-x-rotation)/denominator)* multiplier);
        backRight.setPower(((y+x-rotation)/denominator)* multiplier);


  }

    @Config
    public static final class MecanumDrive {
        public static class Params {
            // IMU orientation
            // TODO: fill in these values based on
            //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
            public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.UP;
            public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            // drive model parameters
            public double inPerTick = 1;
            public double lateralInPerTick = inPerTick;
            public double trackWidthTicks = 0;

            // feedforward parameters (in tick units)
            public double kS = 0;
            public double kV = 0;
            public double kA = 0;

            // path profile parameters (in inches)
            public double maxWheelVel = 50;
            public double minProfileAccel = -30;
            public double maxProfileAccel = 50;

            // turn profile parameters (in radians)
            public double maxAngVel = Math.PI; // shared with path
            public double maxAngAccel = Math.PI;

            // path controller gains
            public double axialGain = 0.0;
            public double lateralGain = 0.0;
            public double headingGain = 0.0; // shared with turn

            public double axialVelGain = 0.0;
            public double lateralVelGain = 0.0;
            public double headingVelGain = 0.0; // shared with turn
        }

        public static Params PARAMS = new Params();

        public final MecanumKinematics kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
        public final VelConstraint defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));
        public final AccelConstraint defaultAccelConstraint =
                new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

        public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

        public final VoltageSensor voltageSensor;

        public final LazyImu lazyImu;

        public final Localizer localizer;
        public Pose2d pose;

        private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

        private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
        private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
        private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
        private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

        public class DriveLocalizer implements Localizer {
            public final Encoder leftFront, leftBack, rightBack, rightFront;
            public final IMU imu;

            private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
            private Rotation2d lastHeading;
            private boolean initialized;

            public DriveLocalizer() {
                leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
                leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
                rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
                rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

                imu = lazyImu.get();

                // TODO: reverse encoders if needed
                //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            @Override
            public Twist2dDual<Time> update() {
                PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
                PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
                PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
                PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

                FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                        leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

                Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

                if (!initialized) {
                    initialized = true;

                    lastLeftFrontPos = leftFrontPosVel.position;
                    lastLeftBackPos = leftBackPosVel.position;
                    lastRightBackPos = rightBackPosVel.position;
                    lastRightFrontPos = rightFrontPosVel.position;

                    lastHeading = heading;

                    return new Twist2dDual<>(
                            Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                            DualNum.constant(0.0, 2)
                    );
                }

                double headingDelta = heading.minus(lastHeading);
                Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                        new DualNum<Time>(new double[]{
                                (leftFrontPosVel.position - lastLeftFrontPos),
                                leftFrontPosVel.velocity,
                        }).times(PARAMS.inPerTick),
                        new DualNum<Time>(new double[]{
                                (leftBackPosVel.position - lastLeftBackPos),
                                leftBackPosVel.velocity,
                        }).times(PARAMS.inPerTick),
                        new DualNum<Time>(new double[]{
                                (rightBackPosVel.position - lastRightBackPos),
                                rightBackPosVel.velocity,
                        }).times(PARAMS.inPerTick),
                        new DualNum<Time>(new double[]{
                                (rightFrontPosVel.position - lastRightFrontPos),
                                rightFrontPosVel.velocity,
                        }).times(PARAMS.inPerTick)
                ));

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new Twist2dDual<>(
                        twist.line,
                        DualNum.cons(headingDelta, twist.angle.drop(1))
                );
            }
        }

        public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
            this.pose = pose;

            LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            // TODO: make sure your config has motors with these names (or change them)
            //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // TODO: reverse motor directions if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

            // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
            //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
            lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                    PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

            voltageSensor = hardwareMap.voltageSensor.iterator().next();

            localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);

            FlightRecorder.write("MECANUM_PARAMS", PARAMS);
        }

        public void setDrivePowers(PoseVelocity2d powers) {
            MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                    PoseVelocity2dDual.constant(powers, 1));

            double maxPowerMag = 1;
            for (DualNum<Time> power : wheelVels.all()) {
                maxPowerMag = Math.max(maxPowerMag, power.value());
            }

            leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
            leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
            rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
            rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
        }

        public final class FollowTrajectoryAction implements Action {
            public final TimeTrajectory timeTrajectory;
            private double beginTs = -1;

            private final double[] xPoints, yPoints;

            public FollowTrajectoryAction(TimeTrajectory t) {
                timeTrajectory = t;

                List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                        0, t.path.length(),
                        Math.max(2, (int) Math.ceil(t.path.length() / 2)));
                xPoints = new double[disps.size()];
                yPoints = new double[disps.size()];
                for (int i = 0; i < disps.size(); i++) {
                    Pose2d p = t.path.get(disps.get(i), 1).value();
                    xPoints[i] = p.position.x;
                    yPoints[i] = p.position.y;
                }
            }

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                double t;
                if (beginTs < 0) {
                    beginTs = Actions.now();
                    t = 0;
                } else {
                    t = Actions.now() - beginTs;
                }

                if (t >= timeTrajectory.duration) {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);

                    return false;
                }

                Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
                targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

                PoseVelocity2d robotVelRobot = updatePoseEstimate();

                PoseVelocity2dDual<Time> command = new HolonomicController(
                        PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                        PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
                )
                        .compute(txWorldTarget, pose, robotVelRobot);
                driveCommandWriter.write(new DriveCommandMessage(command));

                MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
                double voltage = voltageSensor.getVoltage();

                final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                        PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
                double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
                double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
                double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
                double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
                mecanumCommandWriter.write(new MecanumCommandMessage(
                        voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                ));

                leftFront.setPower(leftFrontPower);
                leftBack.setPower(leftBackPower);
                rightBack.setPower(rightBackPower);
                rightFront.setPower(rightFrontPower);

                p.put("x", pose.position.x);
                p.put("y", pose.position.y);
                p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

                Pose2d error = txWorldTarget.value().minusExp(pose);
                p.put("xError", error.position.x);
                p.put("yError", error.position.y);
                p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

                // only draw when active; only one drive action should be active at a time
                Canvas c = p.fieldOverlay();
                drawPoseHistory(c);

                c.setStroke("#4CAF50");
                Drawing.drawRobot(c, txWorldTarget.value());

                c.setStroke("#3F51B5");
                Drawing.drawRobot(c, pose);

                c.setStroke("#4CAF50FF");
                c.setStrokeWidth(1);
                c.strokePolyline(xPoints, yPoints);

                return true;
            }

            @Override
            public void preview(Canvas c) {
                c.setStroke("#4CAF507A");
                c.setStrokeWidth(1);
                c.strokePolyline(xPoints, yPoints);
            }
        }

        public final class TurnAction implements Action {
            private final TimeTurn turn;

            private double beginTs = -1;

            public TurnAction(TimeTurn turn) {
                this.turn = turn;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                double t;
                if (beginTs < 0) {
                    beginTs = Actions.now();
                    t = 0;
                } else {
                    t = Actions.now() - beginTs;
                }

                if (t >= turn.duration) {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);

                    return false;
                }

                Pose2dDual<Time> txWorldTarget = turn.get(t);
                targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

                PoseVelocity2d robotVelRobot = updatePoseEstimate();

                PoseVelocity2dDual<Time> command = new HolonomicController(
                        PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                        PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
                )
                        .compute(txWorldTarget, pose, robotVelRobot);
                driveCommandWriter.write(new DriveCommandMessage(command));

                MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
                double voltage = voltageSensor.getVoltage();
                final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                        PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
                double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
                double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
                double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
                double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
                mecanumCommandWriter.write(new MecanumCommandMessage(
                        voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                ));

                leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
                leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
                rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
                rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

                Canvas c = p.fieldOverlay();
                drawPoseHistory(c);

                c.setStroke("#4CAF50");
                Drawing.drawRobot(c, txWorldTarget.value());

                c.setStroke("#3F51B5");
                Drawing.drawRobot(c, pose);

                c.setStroke("#7C4DFFFF");
                c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

                return true;
            }

            @Override
            public void preview(Canvas c) {
                c.setStroke("#7C4DFF7A");
                c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
            }
        }

        public PoseVelocity2d updatePoseEstimate() {
            Twist2dDual<Time> twist = localizer.update();
            pose = pose.plus(twist.value());

            poseHistory.add(pose);
            while (poseHistory.size() > 100) {
                poseHistory.removeFirst();
            }

            estimatedPoseWriter.write(new PoseMessage(pose));

            return twist.velocity().value();
        }

        private void drawPoseHistory(Canvas c) {
            double[] xPoints = new double[poseHistory.size()];
            double[] yPoints = new double[poseHistory.size()];

            int i = 0;
            for (Pose2d t : poseHistory) {
                xPoints[i] = t.position.x;
                yPoints[i] = t.position.y;

                i++;
            }

            c.setStrokeWidth(1);
            c.setStroke("#3F51B5");
            c.strokePolyline(xPoints, yPoints);
        }

        public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
            return new TrajectoryActionBuilder(
                    TurnAction::new,
                    FollowTrajectoryAction::new,
                    new TrajectoryBuilderParams(
                            1e-6,
                            new ProfileParams(
                                    0.25, 0.1, 1e-2
                            )
                    ),
                    beginPose, 0.0,
                    defaultTurnConstraints,
                    defaultVelConstraint, defaultAccelConstraint
            );
        }
    }

    @Config
    public static final class ThreeDeadWheelLocalizer implements Localizer {
        public static class Params {
            public double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
            public double par1YTicks = 1.0; // y position of the second parallel encoder (in tick units)
            public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
        }

        public static Params PARAMS = new Params();

        public final Encoder par0, par1, perp;

        public final double inPerTick;

        private int lastPar0Pos, lastPar1Pos, lastPerpPos;
        private boolean initialized;

        public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
            // TODO: make sure your config has **motors** with these names (or change them)
            //   the encoders should be plugged into the slot matching the named motor
            //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
            par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0")));
            par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
            perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

            // TODO: reverse encoder directions if needed
            //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

            this.inPerTick = inPerTick;

            FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
        }

        public Twist2dDual<Time> update() {
            PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
            PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
            PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

            FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

            if (!initialized) {
                initialized = true;

                lastPar0Pos = par0PosVel.position;
                lastPar1Pos = par1PosVel.position;
                lastPerpPos = perpPosVel.position;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            int par0PosDelta = par0PosVel.position - lastPar0Pos;
            int par1PosDelta = par1PosVel.position - lastPar1Pos;
            int perpPosDelta = perpPosVel.position - lastPerpPos;

            Twist2dDual<Time> twist = new Twist2dDual<>(
                    new Vector2dDual<>(
                            new DualNum<Time>(new double[] {
                                    (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                    (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                            }).times(inPerTick),
                            new DualNum<Time>(new double[] {
                                    (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                    (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                            }).times(inPerTick)
                    ),
                    new DualNum<>(new double[] {
                            (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                            (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                    })
            );

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return twist;
        }
    }

    @Config
    public static final class TwoDeadWheelLocalizer implements Localizer {
        public static class Params {
            public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
            public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
        }

        public static Params PARAMS = new Params();

        public final Encoder par, perp;
        public final IMU imu;

        private int lastParPos, lastPerpPos;
        private Rotation2d lastHeading;

        private final double inPerTick;

        private double lastRawHeadingVel, headingVelOffset;
        private boolean initialized;

        public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick) {
            // TODO: make sure your config has **motors** with these names (or change them)
            //   the encoders should be plugged into the slot matching the named motor
            //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
            par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
            perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

            // TODO: reverse encoder directions if needed
            //   par.setDirection(DcMotorSimple.Direction.REVERSE);

            this.imu = imu;

            this.inPerTick = inPerTick;

            FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
        }

        public Twist2dDual<Time> update() {
            PositionVelocityPair parPosVel = par.getPositionAndVelocity();
            PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
            AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            AngularVelocity angularVelocity = new AngularVelocity(
                    UnnormalizedAngleUnit.RADIANS,
                    (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                    (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                    (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                    angularVelocityDegrees.acquisitionTime
            );

            FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
            double rawHeadingVel = angularVelocity.zRotationRate;
            if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
                headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
            }
            lastRawHeadingVel = rawHeadingVel;
            double headingVel = headingVelOffset + rawHeadingVel;

            if (!initialized) {
                initialized = true;

                lastParPos = parPosVel.position;
                lastPerpPos = perpPosVel.position;
                lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            int parPosDelta = parPosVel.position - lastParPos;
            int perpPosDelta = perpPosVel.position - lastPerpPos;
            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = new Twist2dDual<>(
                    new Vector2dDual<>(
                            new DualNum<Time>(new double[] {
                                    parPosDelta - PARAMS.parYTicks * headingDelta,
                                    parPosVel.velocity - PARAMS.parYTicks * headingVel,
                            }).times(inPerTick),
                            new DualNum<Time>(new double[] {
                                    perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                    perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                            }).times(inPerTick)
                    ),
                    new DualNum<>(new double[] {
                            headingDelta,
                            headingVel,
                    })
            );

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return twist;
        }
    }

    public static class LocalizationTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            if (ProgrammingBoard5.TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                waitForStart();

                while (opModeIsActive()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));

                    drive.updatePoseEstimate();

                    telemetry.addData("x", drive.pose.position.x);
                    telemetry.addData("y", drive.pose.position.y);
                    telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                    telemetry.update();

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.fieldOverlay().setStroke("#3F51B5");
                    Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }
            } else if (ProgrammingBoard5.TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
                TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

                waitForStart();

                while (opModeIsActive()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    0.0
                            ),
                            -gamepad1.right_stick_x
                    ));

                    drive.updatePoseEstimate();

                    telemetry.addData("x", drive.pose.position.x);
                    telemetry.addData("y", drive.pose.position.y);
                    telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                    telemetry.update();

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.fieldOverlay().setStroke("#3F51B5");
                    Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }
            } else {
                throw new RuntimeException();
            }
        }
    }

    public static final class Drawing {
        private Drawing() {}


        public static void drawRobot(Canvas c, Pose2d t) {
            final double ROBOT_RADIUS = 9;

            c.setStrokeWidth(1);
            c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

            Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
            Vector2d p1 = t.position.plus(halfv);
            Vector2d p2 = p1.plus(halfv);
            c.strokeLine(p1.x, p1.y, p2.x, p2.y);
        }
    }
}
