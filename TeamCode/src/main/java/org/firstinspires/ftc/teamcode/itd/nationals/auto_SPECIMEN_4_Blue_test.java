package org.firstinspires.ftc.teamcode.itd.nationals;

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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;

import java.util.List;


@Disabled
@Autonomous (name = "auto_SPECIMEN_4_Blue_test")

public final class auto_SPECIMEN_4_Blue_test extends LinearOpMode {
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    Limelight3A limelight;
    private final ElapsedTime runtime = new ElapsedTime();
    positions_and_variables pos = new positions_and_variables();
    boolean sample4detected = false;

    public class Lift {
        private final DcMotorEx VSlideF;
        private final DcMotorEx VSlideB;
        private final DigitalChannel limitSwitch;
        private final Servo OArm;

        public Lift(HardwareMap hardwareMap) {
            VSlideF = hardwareMap.get(DcMotorEx.class, "VSlideF");
            VSlideF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VSlideF.setDirection(DcMotorSimple.Direction.REVERSE);

            VSlideB = hardwareMap.get(DcMotorEx.class, "VSlideB");
            VSlideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VSlideB.setDirection(DcMotorSimple.Direction.REVERSE);


            limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);

            OArm = hardwareMap.get(Servo.class, "OArm");
        }


        public class LiftSpecimen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {

                    VSlideF.setPower(1);
                    VSlideB.setPower(1);
                    telemetry.addData("front Position", VSlideF.getCurrentPosition());
                    telemetry.addData("back Position", VSlideB.getCurrentPosition());
                    telemetry.addData("front Power", VSlideF.getPower());
                    telemetry.addData("back Power", VSlideB.getPower());
                    telemetry.update();

                    initialized = true;
                }

                double pos_VSF = VSlideF.getCurrentPosition();
                packet.put("liftPosF", pos_VSF);
                double pos_VSB = VSlideB.getCurrentPosition();
                packet.put("liftPosB", pos_VSB);
                if (pos_VSF < 1300 || pos_VSB > -1300) {
                    return true;
                } else {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);

                    return false;
                }
            }
        }
        public Action liftSpecimen() {
            return new LiftSpecimen();
        }


        public class ScoreSpecimen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {

                    VSlideF.setPower(1);
                    VSlideB.setPower(1);
                    telemetry.addData("front Position", VSlideF.getCurrentPosition());
                    telemetry.addData("back Position", VSlideB.getCurrentPosition());
                    telemetry.addData("front Power", VSlideF.getPower());
                    telemetry.addData("back Power", VSlideB.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos_VSF = VSlideF.getCurrentPosition();
                packet.put("liftPosF", pos_VSF);
                double pos_VSB = VSlideB.getCurrentPosition();
                packet.put("liftPosB", pos_VSB);
                if (pos_VSF < 1800 || pos_VSB > -1800) {
                    return true;
                } else {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);

                    return false;
                }
            }
        }
        public Action ScoreSpecimen() {
            return new ScoreSpecimen();
        }


            public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {

                    VSlideF.setPower(1);
                    VSlideB.setPower(1);
                    telemetry.addData("front Position", VSlideF.getCurrentPosition());
                    telemetry.addData("back Position", VSlideB.getCurrentPosition());
                    telemetry.addData("front Power", VSlideF.getPower());
                    telemetry.addData("back Power", VSlideB.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos_VSF = VSlideF.getCurrentPosition();
                packet.put("liftPosF", pos_VSF);
                double pos_VSB = VSlideB.getCurrentPosition();
                packet.put("liftPosB", pos_VSB);
                if (pos_VSF < 2750 && pos_VSB > -2750) {
                    if  (pos_VSF > 1600 && pos_VSB < -1600){
                        OArm.setPosition(pos.outtake_arm_sample);
                    }
                    return true;
                } else {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }


        public class LifttoMiddle implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    VSlideF.setPower(-1);
                    VSlideB.setPower(-1);
                    telemetry.addData("front Position", VSlideF.getCurrentPosition());
                    telemetry.addData("back Position", VSlideB.getCurrentPosition());
                    telemetry.addData("front Power", VSlideF.getPower());
                    telemetry.addData("back Power", VSlideB.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos_VSF = VSlideF.getCurrentPosition();
                packet.put("liftPosF", pos_VSF);
                double pos_VSB = VSlideB.getCurrentPosition();
                packet.put("liftPosB", pos_VSB);
//                boolean switchPressed = limitSwitch.getState();
                if (pos_VSF > 1500 && pos_VSB < -1500 && limitSwitch.getState()) {
                    return true;
                }
                else {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    return false;
                }
            }
        }
        public Action lifttoMiddle (){
            return new LifttoMiddle();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    VSlideF.setPower(-1);
                    VSlideB.setPower(-1);
                    telemetry.addData("front Position", VSlideF.getCurrentPosition());
                    telemetry.addData("back Position", VSlideB.getCurrentPosition());
                    telemetry.addData("front Power", VSlideF.getPower());
                    telemetry.addData("back Power", VSlideB.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos_VSF = VSlideF.getCurrentPosition();
                packet.put("liftPosF", pos_VSF);
                double pos_VSB = VSlideB.getCurrentPosition();
                packet.put("liftPosB", pos_VSB);
//                boolean switchPressed = limitSwitch.getState();
                if (pos_VSF > 50 && pos_VSB < -50 && limitSwitch.getState()) {
                    return true;
                }
                else {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }

    }


    //OClaw servo class
    public class OuttakeClaw {
        private final Servo OClaw;

        public OuttakeClaw (HardwareMap hardwareMap) {
            OClaw = hardwareMap.get(Servo.class, "OClaw");
        }


        public class OClaw_Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                OClaw.setPosition(pos.outtake_claw_open);
                return false;
            }
        }

        public Action OpenOClaw() {
            return new OClaw_Open();
        }


        public class OClaw_Closed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OClaw.setPosition(pos.outtake_claw_close);
                return false;
            }
        }

        public Action CloseOClaw() {
            return new OClaw_Closed();
        }

    }



    public class OuttakeArm {
        private final Servo OArm;

        public OuttakeArm (HardwareMap hardwareMap) {
            OArm = hardwareMap.get(Servo.class, "OArm");
        }


        public class OArm_Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                OArm.setPosition(pos.outtake_arm_specimenHold);
                return false;
            }
        }
        public Action ExtendOArm() {
            return new OArm_Out();
        }

        public class OArm_Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                OArm.setPosition(pos.outtake_arm_sample);
                return false;
            }
        }
        public Action LiftOArm() {
            return new OArm_Up();
        }


        public class OArm_Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OArm.setPosition(pos.outtake_arm_transfer);
                return false;
            }
        }
        public Action LowerOArm() {
            return new OArm_Down();
        }

    }



    //intake servos class
    public class Intake {
        private final Servo IArmL;
        private final Servo IArmR;
        private final Servo IClaw;
        private final Servo IArmC;
        private final Servo HSlideL;
        private final Servo HSlideR;

        public Intake (HardwareMap hardwareMap) {
            IArmL = hardwareMap.get(Servo.class, "IArmL");
            IArmR = hardwareMap.get(Servo.class, "IArmR");
            IArmC = hardwareMap.get(Servo.class, "IArmC");
            IClaw = hardwareMap.get(Servo.class, "IClaw");
            HSlideL = hardwareMap.get(Servo.class, "HSlideL");
            HSlideR = hardwareMap.get(Servo.class, "HSlideR");
        }


        public class Intake_Transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HSlideL.setPosition(pos.hslide_trans);
                HSlideR.setPosition(1-pos.hslide_trans);
                IArmL.setPosition(pos.intake_arm_trans);
                IArmR.setPosition(1-pos.intake_arm_trans);
                IArmC.setPosition(pos.intake_coax_trans);
                IClaw.setPosition(pos.intake_claw_close); //sample transfer position
                return false;
            }
        }
        public Action SettoTrasfer () {  return new Intake_Transfer();
        }


        public class Intake_AfterTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HSlideL.setPosition(pos.hslide_trans);
                HSlideR.setPosition(1-pos.hslide_trans);
                IArmL.setPosition(pos.intake_arm_trans);
                IArmR.setPosition(1-pos.intake_arm_trans);
                IArmC.setPosition(pos.intake_coax_aftertrans);
                IClaw.setPosition(pos.intake_claw_open); //after transfer position
                return false;
            }
        }
        public Action SettoAfterTrasfer () {
            return new Intake_AfterTransfer();
        }



        public class Intake_Vision implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_vision);
                IArmR.setPosition(1-pos.intake_arm_vision);
                IArmC.setPosition(pos.intake_coax_vision);
                IClaw.setPosition(pos.intake_claw_open); // aiming position
                return false;
            }
        }
        public Action SettoVision () {
            return new Intake_Vision();
        }

        public class Intake_Aim implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_aim);
                IArmR.setPosition(1-pos.intake_arm_aim);
                IArmC.setPosition(pos.intake_coax_aim);
                IClaw.setPosition(pos.intake_claw_open); // aiming position
                return false;
            }
        }
        public Action SettoAim () {
            return new Intake_Aim();
        }


        public class Intake_Grab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_grab);
                IArmR.setPosition(1-pos.intake_arm_grab);
                IArmC.setPosition(pos.intake_coax_grab);
                IClaw.setPosition(pos.intake_claw_close); // grabbing sample
                return false;
            }
        }
        public Action SettoGrab() {

            return new Intake_Grab();
        }


        public class Intake_AfterGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_lift);
                IArmR.setPosition(1-pos.intake_arm_lift);
                IArmC.setPosition(pos.intake_coax_lift);
                IClaw.setPosition(pos.intake_claw_close);// after grabbing specimen
                return false;
            }
        }
        public Action SettoAfterGrab() {
            return new Intake_AfterGrab();
        }

        public class Intake_Drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_aim);
                IArmR.setPosition(1-pos.intake_arm_aim);
                IArmC.setPosition(pos.intake_coax_aim);
                IClaw.setPosition(pos.intake_claw_close);// after grabbing specimen
                return false;
            }
        }
        public Action SettoDrop() {
            return new Intake_Drop();
        }
    }


    //IntakeClaw servo class
    public class IntakeClaw {
        private final Servo IClaw;

        public IntakeClaw (HardwareMap hardwareMap) {
            IClaw = hardwareMap.get(Servo.class, "IClaw");
        }


        public class IClaw_Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IClaw.setPosition(pos.intake_claw_open);
                return false;
            }
        }
        public Action OpenIClaw() {
            return new IClaw_Open();
        }


        public class IClaw_Closed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                IClaw.setPosition(pos.intake_claw_close);
                return false;
            }
        }
        public Action CloseIClaw() {
            return new IClaw_Closed();
        }

    }




    //sample claw wrist servo class
    public class IntakeWrist {
        private final Servo IWrist;
        public IntakeWrist (HardwareMap hardwareMap) {
            IWrist = hardwareMap.get(Servo.class, "IWrist");
            IWrist.scaleRange(0.2, 0.76);
        }


        public class Wrist0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist0);
                return false;
            }
        }
        public Action SettoWrist0() {
            return new Wrist0();
        }


        public class Wrist45 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist45);
                return false;
            }
        }
        public Action SettoWrist45() {
            return new Wrist45();
        }


        public class Wrist90 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist90);
                return false;
            }
        }
        public Action SettoWrist90() {
            return new Wrist90();
        }


        public class Wrist135 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist135);
                return false;
            }
        }
        public Action SettoWrist135() {
            return new Wrist135();
        }


        public class Wrist_Vision implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist_vision);
                return false;
            }
        }
        public Action SettoWrist_Vision() {
            return new Wrist_Vision();
        }

        public class Wrist153 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist153);
                return false;
            }
        }
        public Action SettoWrist153() {
            return new Wrist153();
        }


        public class Wrist180 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist180);
                return false;
            }
        }
        public Action SettoWrist180() {
            return new Wrist180();
        }

    }





    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(-10, 66, (Math.toRadians(90)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Lift lift = new Lift(hardwareMap);
        OuttakeClaw oclaw = new OuttakeClaw(hardwareMap);
        OuttakeArm oarm = new OuttakeArm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        IntakeClaw iclaw = new IntakeClaw(hardwareMap);
        IntakeWrist wrist = new IntakeWrist(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Actions.runBlocking(oclaw.CloseOClaw());
        Actions.runBlocking(oarm.ExtendOArm());
        Actions.runBlocking(intake.SettoAfterTrasfer());
        Actions.runBlocking(iclaw.OpenIClaw());
        Actions.runBlocking(wrist.SettoWrist0());
        DigitalChannel limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
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
        limelight.pipelineSwitch(5); //blue single object
        limelight.start();
        ElapsedTime LLCorrectionTimer = new ElapsedTime();


        //score held specimen
        TrajectoryActionBuilder go_score_specimen_0 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-10, 41.5), (Math.toRadians(90)));

        //go to specimen 1 placed by human player
        TrajectoryActionBuilder go_to_specimen_1 = go_score_specimen_0.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-30, 46), (Math.toRadians(135)));

        //go score specimen 1
        TrajectoryActionBuilder go_score_specimen_1 = go_to_specimen_1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-8, 41.5), (Math.toRadians(90)));

        //go to sample 4
        TrajectoryActionBuilder go_to_sample_4 = go_score_specimen_1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-32, 46), (Math.toRadians(-135)));

        //go drop sample 4
        TrajectoryActionBuilder go_drop_sample_4 = go_to_sample_4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-36, 48), (Math.toRadians(135)));

        //go to sample 5
        TrajectoryActionBuilder go_to_sample_5 = go_drop_sample_4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-40, 44), (Math.toRadians(-135)));

        //go drop sample 5
        TrajectoryActionBuilder go_drop_sample_5 = go_to_sample_5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-36, 48), (Math.toRadians(135)));

        //go to specimen 2 placed by human player
        TrajectoryActionBuilder go_to_specimen_2 = go_to_sample_5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-30, 46), (Math.toRadians(135)));

        //go score specimen 2
        TrajectoryActionBuilder go_score_specimen_2 = go_to_specimen_2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-6, 42.5), (Math.toRadians(90)));

        //go to specimen 3 placed by human player
        TrajectoryActionBuilder go_to_specimen_3 = go_score_specimen_2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-30, 46), (Math.toRadians(135)));

        //go score specimen 3
        TrajectoryActionBuilder go_score_specimen_3 = go_to_specimen_3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-4, 42.5), (Math.toRadians(90)));

        //go park
        TrajectoryActionBuilder go_park = go_score_specimen_3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-30, 56), (Math.toRadians(160)));




        waitForStart();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() <= 0.1) {
            boolean switchPressed = !limitSwitch.getState(); // Inverted to show "pressed" as true
            // Display the state on the telemetry
            if (switchPressed) {
                telemetry.addData("Limit Switch", "Pressed");
            } else {
                telemetry.addData("Limit Switch", "Not Pressed");
            }
            telemetry.update();

            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            go_score_specimen_0.build(),
                            lift.liftSpecimen()
                    ),
                    lift.ScoreSpecimen(),
                    oclaw.OpenOClaw(),

                    // specimen 0 cycle completes by now. specimen 1 cycle starts below.
                    new ParallelAction(
                            go_to_specimen_1.build(),
                            oarm.LowerOArm(),
                            lift.liftDown(),
                            intake.SettoVision(),
                            wrist.SettoWrist_Vision()
                    )
                    ));

            //add limelight movement here
            LLCorrectionTimer.reset();
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid() && result.getTa() > 0.01 && result.getTa() < 0.1 && LLCorrectionTimer.seconds() <= 1) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        if (result.getTy() < -2) {
                            FL.setPower(0.3);
                            BR.setPower(0.3);
                            FR.setPower(-0.3);
                            BL.setPower(-0.3);
                        } else if (result.getTy() > 2) {
                            FL.setPower(-0.3);
                            BR.setPower(-0.3);
                            FR.setPower(0.3);
                            BL.setPower(0.3);
                        } else {
                            if (result.getTx() > 2) {
                                FL.setPower(0.3);
                                BR.setPower(0.3);
                                FR.setPower(0.3);
                                BL.setPower(0.3);
                            } else if (result.getTx() < -2) {
                                FL.setPower(-0.3);
                                BR.setPower(-0.3);
                                FR.setPower(-0.3);
                                BL.setPower(-0.3);
                            } else {
                                FL.setPower(0);
                                BR.setPower(0);
                                FR.setPower(0);
                                BL.setPower(0);
                                telemetry.addData("limelight loop specimen 1 breaks", FL.getPower());
                                telemetry.update();
                                break;
                            }
                        }
                    } else if (result.isValid() && result.getTa() > 0.01 && result.getTa() < 0.1 && LLCorrectionTimer.seconds() > 1) {
                        Actions.runBlocking(
                                go_to_specimen_1.build()
                        );
                        break;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            };
            //limelight correction done
            FL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            drive.updatePoseEstimate();
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            intake.SettoAim(),
                            wrist.SettoWrist0()
                    ),
                    new SleepAction(0.4),
                    intake.SettoGrab(),
                    new SleepAction(0.3),
                    new ParallelAction(
                            go_score_specimen_1.build(),
                            new SequentialAction(
                                    intake.SettoAfterGrab(),
                                    new SleepAction(0.2),
                                    wrist.SettoWrist180(),
                                    new SleepAction(0.2),
                                    intake.SettoTrasfer(),
                                    new SleepAction(0.6),
                                    oclaw.CloseOClaw(),
                                    new SleepAction(0.3),
                                    iclaw.OpenIClaw(),
                                    new ParallelAction(
                                            intake.SettoAfterTrasfer(),
                                            oarm.ExtendOArm()
                                    )
                            )
                    ),
                    new SleepAction(0.4),
                    lift.ScoreSpecimen(),
                    oclaw.OpenOClaw(),

                    // specimen 1 cycle completes by now. sample 4 cycle starts below.
                    new ParallelAction(
                            go_to_sample_4.build(),
                            oarm.LowerOArm(),
                            lift.liftDown(),
                            intake.SettoVision(),
                            wrist.SettoWrist_Vision()
                    )
                    ));
//add limelight movement here
            LLCorrectionTimer.reset();
            while (opModeIsActive() && !isStopRequested()) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid() && result.getTa() > 0.001 && LLCorrectionTimer.seconds() <= 1) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        if (result.getTy() < -2) {
                            FL.setPower(0.3);
                            BR.setPower(0.3);
                            FR.setPower(-0.3);
                            BL.setPower(-0.3);
                        } else if (result.getTy() > 2) {
                            FL.setPower(-0.3);
                            BR.setPower(-0.3);
                            FR.setPower(0.3);
                            BL.setPower(0.3);
                        } else {
                            if (result.getTx() > 2) {
                                FL.setPower(0.3);
                                BR.setPower(0.3);
                                FR.setPower(0.3);
                                BL.setPower(0.3);
                            } else if (result.getTx() < -2) {
                                FL.setPower(-0.3);
                                BR.setPower(-0.3);
                                FR.setPower(-0.3);
                                BL.setPower(-0.3);
                            } else {
                                FL.setPower(0);
                                BR.setPower(0);
                                FR.setPower(0);
                                BL.setPower(0);
                                telemetry.addData("limelight loop 2 breaks", FL.getPower());
                                telemetry.update();
                                break;
                            }
                        }
                    } else if (result.isValid() && result.getTa() > 0.001 && LLCorrectionTimer.seconds() > 1) {
                        Actions.runBlocking(
                                go_to_sample_4.build()
                        );
                        break;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            };
            //limelight correction done
            FL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);

            drive.updatePoseEstimate();
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            intake.SettoAim(),
                            wrist.SettoWrist45()
                    ),
                    new SleepAction(0.4),
                    intake.SettoGrab(),
                    new SleepAction(0.3),
                    new ParallelAction(
                            go_drop_sample_4.build(),
                            intake.SettoDrop()
                    ),
                    new SleepAction(0.6),
                    intake.SettoAim(),
                    new SleepAction(0.4),
                    // sample 4 cycle completes by now. sample 5 cycle starts below.

                    new ParallelAction(
                            go_to_sample_5.build(),
                            intake.SettoVision(),
                            wrist.SettoWrist_Vision()
                    )
            ));
                    //add limelight movement here
            LLCorrectionTimer.reset();
            while (opModeIsActive() && !isStopRequested()) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid() && result.getTa() > 0.001 && LLCorrectionTimer.seconds() <= 1) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        if (result.getTy() < -2) {
                            FL.setPower(0.3);
                            BR.setPower(0.3);
                            FR.setPower(-0.3);
                            BL.setPower(-0.3);
                        } else if (result.getTy() > 2) {
                            FL.setPower(-0.3);
                            BR.setPower(-0.3);
                            FR.setPower(0.3);
                            BL.setPower(0.3);
                        } else {
                            if (result.getTx() > 2) {
                                FL.setPower(0.3);
                                BR.setPower(0.3);
                                FR.setPower(0.3);
                                BL.setPower(0.3);
                            } else if (result.getTx() < -2) {
                                FL.setPower(-0.3);
                                BR.setPower(-0.3);
                                FR.setPower(-0.3);
                                BL.setPower(-0.3);
                            } else {
                                FL.setPower(0);
                                BR.setPower(0);
                                FR.setPower(0);
                                BL.setPower(0);
                                telemetry.addData("limelight loop 2 breaks", FL.getPower());
                                telemetry.update();
                                break;
                            }
                        }
                    } else if (result.isValid() && result.getTa() > 0.001 && LLCorrectionTimer.seconds() > 1) {
                        Actions.runBlocking(
                                go_to_sample_5.build()
                        );
                        break;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            };
            //limelight correction done
            FL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);

            drive.updatePoseEstimate();
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            intake.SettoAim(),
                            wrist.SettoWrist45()
                    ),
                    new SleepAction(0.4),
                    intake.SettoGrab(),
                    new SleepAction(0.3),
                    new ParallelAction(
                            go_drop_sample_4.build(),
                            intake.SettoDrop()
                    ),
                    new SleepAction(0.6),
                    intake.SettoAim(),
                    new SleepAction(0.4),




                    // sample 5 cycle completes by now. specimen 2 cycle starts below.
                    new ParallelAction(
                            go_to_specimen_2.build(),
                            intake.SettoVision(),
                            wrist.SettoWrist_Vision()
                    ),
                    new SleepAction(0.5)
                    ));

            //add limelight movement here
            LLCorrectionTimer.reset();
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid() && result.getTa() > 0.01 && LLCorrectionTimer.seconds() <= 1) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        if (result.getTy() < -2) {
                            FL.setPower(0.3);
                            BR.setPower(0.3);
                            FR.setPower(-0.3);
                            BL.setPower(-0.3);
                        } else if (result.getTy() > 2) {
                            FL.setPower(-0.3);
                            BR.setPower(-0.3);
                            FR.setPower(0.3);
                            BL.setPower(0.3);
                        } else {
                            if (result.getTx() > 2) {
                                FL.setPower(0.3);
                                BR.setPower(0.3);
                                FR.setPower(0.3);
                                BL.setPower(0.3);
                            } else if (result.getTx() < -2) {
                                FL.setPower(-0.3);
                                BR.setPower(-0.3);
                                FR.setPower(-0.3);
                                BL.setPower(-0.3);
                            } else {
                                FL.setPower(0);
                                BR.setPower(0);
                                FR.setPower(0);
                                BL.setPower(0);
                                telemetry.addData("limelight loop sample 5 breaks", FL.getPower());
                                telemetry.update();
                                break;
                            }
                        }
                    } else if (result.isValid() && result.getTa() > 0.01 && LLCorrectionTimer.seconds() > 1) {
                        Actions.runBlocking(
                                go_to_specimen_2.build()
                        );
                        break;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            };
            //limelight correction done
            FL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);

            drive.updatePoseEstimate();
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            intake.SettoAim(),
                            wrist.SettoWrist0()
                    ),
                    new SleepAction(0.4),
                    intake.SettoGrab(),
                    new SleepAction(0.3),
                    new ParallelAction(
                            go_score_specimen_2.build(),
                            new SequentialAction(
                                    intake.SettoAfterGrab(),
                                    new SleepAction(0.2),
                                    wrist.SettoWrist180(),
                                    new SleepAction(0.2),
                                    intake.SettoTrasfer(),
                                    new SleepAction(0.6),
                                    oclaw.CloseOClaw(),
                                    new SleepAction(0.3),
                                    iclaw.OpenIClaw(),
                                    new ParallelAction(
                                            intake.SettoAfterTrasfer(),
                                            oarm.ExtendOArm()
                                    )
                            )
                    ),
                    new SleepAction(0.3),
                    lift.ScoreSpecimen(),
                    oclaw.OpenOClaw(),


                    // specimen 2 cycle completes by now. specimen 3 cycle starts below.
                    new ParallelAction(
                            go_to_specimen_3.build(),
                            oarm.LowerOArm(),
                            lift.liftDown(),
                            intake.SettoVision(),
                            wrist.SettoWrist_Vision()
                    ),
                    new SleepAction(0.5)
            ));

            //add limelight movement here
            LLCorrectionTimer.reset();
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid() && result.getTa() > 0.01 && LLCorrectionTimer.seconds() <= 1) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        if (result.getTy() < -2) {
                            FL.setPower(0.3);
                            BR.setPower(0.3);
                            FR.setPower(-0.3);
                            BL.setPower(-0.3);
                        } else if (result.getTy() > 2) {
                            FL.setPower(-0.3);
                            BR.setPower(-0.3);
                            FR.setPower(0.3);
                            BL.setPower(0.3);
                        } else {
                            if (result.getTx() > 2) {
                                FL.setPower(0.3);
                                BR.setPower(0.3);
                                FR.setPower(0.3);
                                BL.setPower(0.3);
                            } else if (result.getTx() < -2) {
                                FL.setPower(-0.3);
                                BR.setPower(-0.3);
                                FR.setPower(-0.3);
                                BL.setPower(-0.3);
                            } else {
                                FL.setPower(0);
                                BR.setPower(0);
                                FR.setPower(0);
                                BL.setPower(0);
                                telemetry.addData("limelight loop sample 5 breaks", FL.getPower());
                                telemetry.update();
                                break;
                            }
                        }
                    } else if (result.isValid() && result.getTa() > 0.01 && LLCorrectionTimer.seconds() > 1) {
                        Actions.runBlocking(
                                go_to_specimen_3.build()
                        );
                        break;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            };
            //limelight correction done
            FL.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);

            drive.updatePoseEstimate();
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            intake.SettoAim(),
                            wrist.SettoWrist0()
                    ),
                    new SleepAction(0.4),
                    intake.SettoGrab(),
                    new SleepAction(0.3),
                    new ParallelAction(
                            go_score_specimen_3.build(),
                            new SequentialAction(
                                    intake.SettoAfterGrab(),
                                    new SleepAction(0.2),
                                    wrist.SettoWrist180(),
                                    new SleepAction(0.2),
                                    intake.SettoTrasfer(),
                                    new SleepAction(0.6),
                                    oclaw.CloseOClaw(),
                                    new SleepAction(0.3),
                                    iclaw.OpenIClaw(),
                                    new ParallelAction(
                                            intake.SettoAfterTrasfer(),
                                            oarm.ExtendOArm()
                                    )
                            )
                    ),
                    new SleepAction(0.3),
                    lift.ScoreSpecimen(),
                    oclaw.OpenOClaw(),

                    // specimen 3 cycle completes by now. go park
                    new ParallelAction(
                            go_park.build(),
                            oarm.LowerOArm(),
                            lift.liftDown(),
                            intake.SettoVision(),
                            wrist.SettoWrist_Vision()
                    )
            ));
        }
    }
}


