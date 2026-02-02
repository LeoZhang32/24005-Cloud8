package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous (name = "decode auto red far 21 22")

public final class auto_red_far_21_22 extends LinearOpMode {
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    Limelight3A limelight;
    private final ElapsedTime runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private static final int DESIRED_TAG_ID21 = 21;
    private static final int DESIRED_TAG_ID22 = 22;
    private static final int DESIRED_TAG_ID23 = 23;
    private AprilTagDetection desiredTag21 = null;
    private AprilTagDetection desiredTag22 = null;
    private AprilTagDetection desiredTag23 = null;
    boolean target21Found = false;
    boolean target22Found = false;
    boolean target23Found = false;
    boolean targetAligned = false;
    double integralSum = 0;
    double Kp = 0.0325;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0.0032;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

        public class Intake {
        private final DcMotorEx intake;
        private CRServo intakeCR;

            public Intake (HardwareMap hardwareMap) {
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                intakeCR = hardwareMap.get(CRServo.class, "intakeCR");
                intakeCR.setDirection(CRServo.Direction.REVERSE);
            }

        //IntakeRun Function
            public class IntakeRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setPower(1);
                    return false;
                }
            }
            public Action IntakeRun() {
                return new IntakeRun();
            }

            //IntakeCR Function
            public class IntakeCRRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeCR.setPower(1);
                    return false;
                }
            }
            public Action IntakeCRRun() {
                return new IntakeCRRun();
            }


            //IntakeStop Function
            public class IntakeStop implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setPower(0);
                    intakeCR.setPower(0);
                    return false;
                }
            }
            public Action IntakeStop() {
                return new IntakeStop();
            }


        }




        public class Outtake {
            private final DcMotorEx shooterTop;
            private final DcMotorEx shooterBottom;
            private final DcMotorEx intake;
            private Servo trigger;
            private CRServo intakeCR;
            boolean shooterStopped = false;
            ElapsedTime shootertimer = new ElapsedTime();

            public Outtake (HardwareMap hardwareMap) {
                shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
                shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
                shooterTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                shooterBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                intakeCR = hardwareMap.get(CRServo.class, "intakeCR");
                intakeCR.setDirection(CRServo.Direction.REVERSE);
                trigger = hardwareMap.get(Servo.class, "trigger");
                shootertimer.reset();
            }
            public class OuttakeTimerReset implements Action{
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket){
                    shootertimer.reset();
                    return false;
                }
            }
            public Action OuttakeTimerReset() {return new OuttakeTimerReset();}
            //OuttakeRun Function
            public class OuttakeRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    double power =  PIDControl(190, shooterTop.getVelocity(AngleUnit.DEGREES));
                    shooterTop.setPower(power);
                    shooterBottom.setPower(power);
                    if (shootertimer.seconds() >= 1.3){
                        trigger.setPosition(0.68);
                        intake.setPower(1);
                        intakeCR.setPower(1);
                    }
                    if (shootertimer.seconds() >= 3.6){
                        shootertimer.reset();
                        return false;
                    }
                    else {return true;}
                }
            }
            public Action OuttakeRun() {
                return new OuttakeRun();
            }
            //OuttakeStop Function
            public class OuttakeIdle implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    shooterStopped = true;
                    shooterTop.setPower(0.4);
                    shooterBottom.setPower(0.4);
                    intake.setPower(0);
                    intakeCR.setPower(0);
                    trigger.setPosition(0.95);
                    return false;
                }
            }
            public Action OuttakeIdle() {
                return new OuttakeIdle();
            }

            //OuttakeStop Function
            public class OuttakeStop implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    shooterStopped = true;
                    shooterTop.setPower(0);
                    shooterBottom.setPower(0);
                    intake.setPower(0);
                    intakeCR.setPower(0);
                    trigger.setPosition(0.95);
                    return false;
                }
            }
            public Action OuttakeStop() {
                return new OuttakeStop();
            }




        }


    //trigger servo class
    public class Trigger {
        private final Servo trigger;

        public Trigger (HardwareMap hardwareMap) {
            trigger = hardwareMap.get(Servo.class, "trigger");
        }


        public class Trigger_Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                trigger.setPosition(0.68);
                return false;
            }
        }

        public Action OpenTrigger() {
            return new Trigger_Open();
        }


        public class Trigger_Closed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                trigger.setPosition(0.95);
                return false;
            }
        }

        public Action CloseTrigger() {
            return new Trigger_Closed();
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {




        Pose2d beginPose = new Pose2d(63, 9, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);
        Actions.runBlocking(trigger.CloseTrigger());
        // drivetrain motors
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(7);
        limelight.start();
        ElapsedTime LLCorrectionTimer = new ElapsedTime();


        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
        if (USE_WEBCAM){
            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() &&(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested()) {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long) 1, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(250);
                sleep(20);
            }
        }

        if (visionPortal.getCameraState() != null) {
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
            }
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            sleep(20);
        }

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();



//
//        //go scan obelisk
//        TrajectoryActionBuilder go_scan_obelisk = driveRobot.actionBuilder(beginPose)
//                .strafeToLinearHeading(new Vector2d(60, 20), (Math.toRadians(-178)));

        //score held artifacts
        TrajectoryActionBuilder go_shoot_held_artifacts = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(59, 20), (Math.toRadians(156.5)));




//        //After Scan, if it is April Tag 23 (PPG), then follow this sequence: 23PPG
//
//        // go to PPG
//        TrajectoryActionBuilder go_from_obelisk_to_PPG = go_shoot_held_artifacts.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-12, 28), (Math.toRadians(-90)));
//
//        //go collect PPG
//        TrajectoryActionBuilder go_collect_PPG = go_from_obelisk_to_PPG.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-12, 53), Math.toRadians(-90));
//
//        //go shoot PPG
//        TrajectoryActionBuilder go_shoot_PPG = go_collect_PPG.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-12, 17), (Math.toRadians(136)));
//
//        //go from shoot position to PGP
//        TrajectoryActionBuilder go_from_shoot_to_PGP2 = go_shoot_PPG.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(14, 28), (Math.toRadians(-90)));
//
//        //go collect PGP2
//        TrajectoryActionBuilder go_collect_PGP2 = go_from_shoot_to_PGP2.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(14, 61), Math.toRadians(-90));
//
//        //go shoot PPG
//        TrajectoryActionBuilder go_shoot_PGP2 = go_collect_PGP2.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-12, 17), (Math.toRadians(136)));
//
//        //LEAVE
//        TrajectoryActionBuilder go_leave_PGP2 = go_shoot_PGP2.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(0, 20), (Math.toRadians(-135)));
//
//
//
//
//
        //After Scan, if it is April Tag 22 (PGP), then follow this sequence: 22PGP

        // go to PGP
        TrajectoryActionBuilder go_from_shoot_to_PGP2 = go_shoot_held_artifacts.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 28), (Math.toRadians(-90)));

        //go collect PGP
        TrajectoryActionBuilder go_collect_PGP2 = go_from_shoot_to_PGP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 64), Math.toRadians(-90));

        //go shoot PGP
        TrajectoryActionBuilder go_shoot_PGP2 = go_collect_PGP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15, 22), (Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(59, 20), (Math.toRadians(160)));

        //go from shoot position to GPP2
        TrajectoryActionBuilder go_from_shoot_to_GPP2 = go_shoot_PGP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, 28), (Math.toRadians(-90)));

        //go collect GPP2
        TrajectoryActionBuilder go_collect_GPP2 = go_from_shoot_to_GPP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, 64), Math.toRadians(-90));

        //go shoot GPP2
        TrajectoryActionBuilder go_shoot_GPP2 = go_collect_GPP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(59, 20), (Math.toRadians(159)));

        //LEAVE
        TrajectoryActionBuilder go_leave_GPP2 = go_shoot_GPP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(54, 57), (Math.toRadians(-90)));

        //go collect loading PGP
        TrajectoryActionBuilder go_collect_loading_1_PGP = go_leave_GPP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(57, 60), Math.toRadians(-100));

        //go collect loading 2 PGP
        TrajectoryActionBuilder go_collect_loading_2_PGP = go_collect_loading_1_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(62, 61.5), Math.toRadians(-100));

        //go collect loading 3 PGP
        TrajectoryActionBuilder go_collect_loading_3_PGP = go_collect_loading_2_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-90));





        //After Scan, if it is April Tag 21 (GPP), then follow this sequence: GPP

        // go to GPP
        TrajectoryActionBuilder go_from_shoot_to_GPP = go_shoot_held_artifacts.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(33, 28), (Math.toRadians(-90)));

        //go collect GPP
        TrajectoryActionBuilder go_collect_GPP = go_from_shoot_to_GPP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(33, 62.5), Math.toRadians(-90));

        //go shoot GPP
        TrajectoryActionBuilder go_shoot_GPP = go_collect_GPP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(59, 20), (Math.toRadians(159)));

        //go from shoot position to PGP
        TrajectoryActionBuilder go_from_shoot_to_PGP = go_shoot_GPP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 28), (Math.toRadians(-90)));

        //go collect PGP
        TrajectoryActionBuilder go_collect_PGP = go_from_shoot_to_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 62.5), Math.toRadians(-100));

        //go shoot PGP
        TrajectoryActionBuilder go_shoot_PGP = go_collect_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(59, 20), (Math.toRadians(162)));

        //LEAVE
        TrajectoryActionBuilder go_leave_PGP = go_shoot_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(54, 57), (Math.toRadians(-90)));

        //go collect loading GPP
        TrajectoryActionBuilder go_collect_loading_1_GPP = go_leave_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(57, 60), Math.toRadians(-100));

        //go collect loading 2 GPP
        TrajectoryActionBuilder go_collect_loading_2_GPP = go_collect_loading_1_GPP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(62, 61.5), Math.toRadians(-100));

        //go collect loading 3 GPP
        TrajectoryActionBuilder go_collect_loading_3_GPP = go_collect_loading_2_GPP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-90));

        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= 0.01 && !isStopRequested()) {


            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == DESIRED_TAG_ID21) {
                        // Yes, we want to use this tag.
                        target21Found = true;
                        desiredTag21 = detection;
                        telemetry.addData("Tag Detected", detection.id);
                        break;  // don't look any further.
                    } else if (detection.id == DESIRED_TAG_ID22){
                        // Yes, we want to use this tag.
                        target22Found = true;
                        desiredTag22 = detection;
                        telemetry.addData("Tag Detected", detection.id);
                        break;  // don't look any further.
                    } else if (detection.id == DESIRED_TAG_ID23){
                        // Yes, we want to use this tag.
                        target23Found = true;
                        desiredTag23 = detection;
                        telemetry.addData("Tag Detected", detection.id);
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            Actions.runBlocking(new SequentialAction(

//                            go_scan_obelisk.build(),
                            go_shoot_held_artifacts.build(),
                            outtake.OuttakeTimerReset(),
                            outtake.OuttakeRun(),
                            outtake.OuttakeIdle()

                    )
            );

            if (target21Found == true) { //GPP then PGP
                Actions.runBlocking(new SequentialAction(
                    go_from_shoot_to_GPP.build(),
                    intake.IntakeRun(),
                    go_collect_GPP.build(),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                intake.IntakeStop()
                                        ),
                                        go_shoot_GPP.build()
                                ),
                    outtake.OuttakeTimerReset(),
                    outtake.OuttakeRun(),
                    outtake.OuttakeIdle(),

                    //go collect and shoot PGP
                    go_from_shoot_to_PGP.build(),
                    intake.IntakeRun(),
                    go_collect_PGP.build(),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        intake.IntakeStop()
                                ),
                                go_shoot_PGP.build()
                        ),
                    outtake.OuttakeTimerReset(),
                    outtake.OuttakeRun(),
                    outtake.OuttakeStop(),
                    go_leave_PGP.build(),
                    intake.IntakeRun(),
                    go_collect_loading_1_GPP.build(),
                    go_collect_loading_2_GPP.build(),
                    go_collect_loading_3_GPP.build()
                    )
                );

            } else if  (target22Found == true) { //PGP2 then GPP2

                Actions.runBlocking(new SequentialAction(
                                go_from_shoot_to_PGP2.build(),
                                intake.IntakeRun(),
                                go_collect_PGP2.build(),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                intake.IntakeStop()
                                        ),
                                        go_shoot_PGP2.build()
                                ),
                                outtake.OuttakeTimerReset(),
                                outtake.OuttakeRun(),
                                outtake.OuttakeIdle(),

                                //go collect and shoot GPP2
                                go_from_shoot_to_GPP2.build(),
                                intake.IntakeRun(),
                                go_collect_GPP2.build(),
                                new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        intake.IntakeStop()
                                        ),
                                go_shoot_GPP2.build()
                                ),
                                outtake.OuttakeTimerReset(),
                                outtake.OuttakeRun(),
                                outtake.OuttakeStop(),
                                go_leave_GPP2.build(),
                                intake.IntakeRun(),
                                go_collect_loading_1_PGP.build(),
                                go_collect_loading_2_PGP.build(),
                                go_collect_loading_3_PGP.build()
                        )
                );

            } else if  (target23Found == true) { //still GPP then PGP

                Actions.runBlocking(new SequentialAction(
                                go_from_shoot_to_GPP.build(),
                                intake.IntakeRun(),
                                go_collect_GPP.build(),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                intake.IntakeStop()
                                        ),
                                        go_shoot_GPP.build()
                                ),
                                outtake.OuttakeTimerReset(),
                                outtake.OuttakeRun(),
                                outtake.OuttakeIdle(),

                                //go collect and shoot PGP
                                go_from_shoot_to_PGP.build(),
                                intake.IntakeRun(),
                                go_collect_PGP.build(),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                intake.IntakeStop()
                                        ),
                                        go_shoot_PGP.build()
                                ),
                                outtake.OuttakeTimerReset(),
                                outtake.OuttakeRun(),
                                outtake.OuttakeStop(),
                                go_leave_PGP.build(),
                                intake.IntakeRun(),
                                go_collect_loading_1_GPP.build(),
                                go_collect_loading_2_GPP.build(),
                                go_collect_loading_3_GPP.build()
                        )
                );
            } else {//still GPP then PGP

                Actions.runBlocking(new SequentialAction(
                                go_from_shoot_to_GPP.build(),
                                intake.IntakeRun(),
                                go_collect_GPP.build(),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                intake.IntakeStop()
                                        ),
                                        go_shoot_GPP.build()
                                ),
                                outtake.OuttakeTimerReset(),
                                outtake.OuttakeRun(),
                                outtake.OuttakeIdle(),

                                //go collect and shoot PGP
                                go_from_shoot_to_PGP.build(),
                                intake.IntakeRun(),
                                go_collect_PGP.build(),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                intake.IntakeStop()
                                        ),
                                        go_shoot_PGP.build()
                                ),
                                outtake.OuttakeTimerReset(),
                                outtake.OuttakeRun(),
                                outtake.OuttakeStop(),
                                go_leave_PGP.build(),
                                intake.IntakeRun(),
                                go_collect_loading_1_GPP.build(),
                                go_collect_loading_2_GPP.build(),
                                go_collect_loading_3_GPP.build()
                        )
                );
            }
        }
    }
    public double PIDControl (double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();

        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}


