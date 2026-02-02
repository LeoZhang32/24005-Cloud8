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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous (name = "auto_test4")

public final class auto_test4 extends LinearOpMode {
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
            public Outtake (HardwareMap hardwareMap) {

                shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
                shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
                shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                intakeCR = hardwareMap.get(CRServo.class, "intakeCR");
                trigger = hardwareMap.get(Servo.class, "trigger");

            }

            //OuttakeRun Function
            public class OuttakeRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    shooterTop.setPower(0.6);
                    shooterBottom.setPower(0.6);
                    return false;
                }
            }
            public Action OuttakeRun() {
                return new OuttakeRun();
            }



            //OuttakeStop Function
            public class OuttakeStop implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    shooterTop.setPower(0);
                    shooterBottom.setPower(0);
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




        Pose2d beginPose = new Pose2d(-40.5, 57, Math.toRadians(90));
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
        ElapsedTime ShooterTimer = new ElapsedTime();

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




        //score held artifacts
        TrajectoryActionBuilder go_shoot_held_artifacts = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-11, 16), (Math.toRadians(136)));

        //go scan obelisk
        TrajectoryActionBuilder go_scan_obelisk = go_shoot_held_artifacts.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-10, 14), (Math.toRadians(-175)));





        //After Scan, if it is April Tag 23 (PPG), then follow this sequence: PPG

        // go to PPG
        TrajectoryActionBuilder go_from_obelisk_to_PPG = go_scan_obelisk.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, 28), (Math.toRadians(-90)));

        //go collect PPG
        TrajectoryActionBuilder go_collect_PPG = go_from_obelisk_to_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, 48), Math.toRadians(-90));

        //go shoot PPG
        TrajectoryActionBuilder go_shoot_PPG = go_collect_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-11, 16), (Math.toRadians(136)));

        //LEAVE
        TrajectoryActionBuilder go_leave_PPG = go_shoot_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, 18), (Math.toRadians(135)));





        //After Scan, if it is April Tag 22 (PGP), then follow this sequence: PGP

        // go to PGP
        TrajectoryActionBuilder go_from_obelisk_to_PGP = go_scan_obelisk.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 28), (Math.toRadians(-90)));

        //go collect PGP
        TrajectoryActionBuilder go_collect_PGP = go_from_obelisk_to_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 56), Math.toRadians(-90));

        //go shoot PPG
        TrajectoryActionBuilder go_shoot_PGP = go_collect_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 36), (Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-11, 16), (Math.toRadians(136)));

        //LEAVE
        TrajectoryActionBuilder go_leave_PGP = go_shoot_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, 18), (Math.toRadians(135)));





        //After Scan, if it is April Tag 21 (GPP), then follow this sequence: GPP

        // go to GPP
        TrajectoryActionBuilder go_from_obelisk_to_GPP = go_scan_obelisk.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(36, 28), (Math.toRadians(-90)));

        //go collect GPP
        TrajectoryActionBuilder go_collect_GPP = go_from_obelisk_to_GPP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(36, 56), Math.toRadians(-90));

        //go shoot GPP
        TrajectoryActionBuilder go_shoot_GPP = go_collect_GPP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-11, 16), (Math.toRadians(136)));

        //LEAVE
        TrajectoryActionBuilder go_leave_GPP = go_shoot_GPP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, 18), (Math.toRadians(135)));


        waitForStart();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() <= 0.1 && !isStopRequested()) {
            Actions.runBlocking(new SequentialAction(
                    outtake.OuttakeRun(),
                    go_shoot_held_artifacts.build(),
                    new SleepAction(0.5),
                    trigger.OpenTrigger(),
                    new SleepAction(0.5),
                    //first artifact
                    intake.IntakeRun(),
                    intake.IntakeCRRun(),
                    new SleepAction(0.4),
                    intake.IntakeStop(),
                    new SleepAction(0.8),
                    //second artifact
                    intake.IntakeRun(),
                    intake.IntakeCRRun(),
                    new SleepAction(0.7),
                    intake.IntakeStop(),
                    new SleepAction(0.8),
                    //third artifact
                    intake.IntakeRun(),
                    intake.IntakeCRRun(),
                    new SleepAction(1),
                    intake.IntakeStop(),
                    new SleepAction(0.4),
                    trigger.CloseTrigger(),
//                    new SleepAction(1.2),
                    outtake.OuttakeStop(),
                    go_scan_obelisk.build()
//                    new SleepAction(0.1)
            )
            );


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



            if (target21Found == true) { //GPP

                Actions.runBlocking(new SequentialAction(
                                go_from_obelisk_to_GPP.build(),
                                new ParallelAction(
                                        go_collect_GPP.build(),
                                        intake.IntakeRun()
                                ),
                                new SleepAction(2),
                                intake.IntakeStop(),
                                go_shoot_GPP.build(),
                                outtake.OuttakeRun(),
                                new SleepAction(1.5),
                                trigger.OpenTrigger(),
                                new SleepAction(1),
                                //first artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(0.6),
                                intake.IntakeStop(),
                                new SleepAction(1.2),
                                //second artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(0.8),
                                intake.IntakeStop(),
                                new SleepAction(1.2),
                                //third artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(1),
                                intake.IntakeStop(),
                                new SleepAction(1),
                                trigger.CloseTrigger(),
                                new SleepAction(1.2),
                                outtake.OuttakeStop(),
                                go_leave_GPP.build()
                        )
                );

            } else if  (target22Found == true) { //PGP


                Actions.runBlocking(new SequentialAction(
                        go_from_obelisk_to_PGP.build(),
                                intake.IntakeRun(),
                                go_collect_PGP.build(),
                        outtake.OuttakeRun(),
                                new SleepAction(0.6),
                                intake.IntakeStop(),

                                go_shoot_PGP.build(),
                                new SleepAction(0.5),
                                trigger.OpenTrigger(),
                                new SleepAction(0.5),
                                //first artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(0.4),
                                intake.IntakeStop(),
                                new SleepAction(0.8),
                                //second artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(0.7),
                                intake.IntakeStop(),
                                new SleepAction(0.8),
                                //third artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(1),
                                intake.IntakeStop(),
                                new SleepAction(0.4),
                                trigger.CloseTrigger(),
//                                new SleepAction(1.2),
                                outtake.OuttakeStop(),
                                go_leave_PGP.build()
                        )
                );
            } else if  (target23Found == true) { //PPG


                Actions.runBlocking(new SequentialAction(
                        go_from_obelisk_to_PPG.build(),
                        new ParallelAction(
                                go_collect_PPG.build(),
                                intake.IntakeRun()
                        ),
                        new SleepAction(2),
                        intake.IntakeStop(),
                        go_shoot_PPG.build(),
                        outtake.OuttakeRun(),
                        new SleepAction(1.5),
                        trigger.OpenTrigger(),
                        new SleepAction(1),
                        //first artifact
                        intake.IntakeRun(),
                        intake.IntakeCRRun(),
                        new SleepAction(0.6),
                        intake.IntakeStop(),
                        new SleepAction(1.2),
                        //second artifact
                        intake.IntakeRun(),
                        intake.IntakeCRRun(),
                        new SleepAction(0.8),
                        intake.IntakeStop(),
                        new SleepAction(1.2),
                        //third artifact
                        intake.IntakeRun(),
                        intake.IntakeCRRun(),
                        new SleepAction(1),
                        intake.IntakeStop(),
                        new SleepAction(1),
                        trigger.CloseTrigger(),
                        new SleepAction(1.2),
                        outtake.OuttakeStop(),
                        go_leave_PPG.build()
                        )
                );
            } else {

                Actions.runBlocking(new SequentialAction(
                                go_from_obelisk_to_PPG.build(),
                                new ParallelAction(
                                        go_collect_PPG.build(),
                                        intake.IntakeRun()
                                ),
                                new SleepAction(2),
                                intake.IntakeStop(),
                                go_shoot_PPG.build(),
                                outtake.OuttakeRun(),
                                new SleepAction(1.5),
                                trigger.OpenTrigger(),
                                new SleepAction(1),
                                //first artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(0.6),
                                intake.IntakeStop(),
                                new SleepAction(1.2),
                                //second artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(0.8),
                                intake.IntakeStop(),
                                new SleepAction(1.2),
                                //third artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(1),
                                intake.IntakeStop(),
                                new SleepAction(1),
                                trigger.CloseTrigger(),
                                new SleepAction(1.2),
                                outtake.OuttakeStop(),
                                go_leave_PPG.build()
                        )
                );
            }

        }
    }
}


