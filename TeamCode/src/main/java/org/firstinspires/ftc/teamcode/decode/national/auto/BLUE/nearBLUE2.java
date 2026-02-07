package org.firstinspires.ftc.teamcode.decode.national.auto.BLUE; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.national.hardware.color_sensor_hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;

@Autonomous(name = "AAA NEAR BLUE 2", group = "AAA BLUE")
public class nearBLUE2 extends CommandOpMode {
    private Follower follower;

    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;
    DcMotorEx turretEncoder;
    DcMotorEx intake;
    CRServo turret;
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    double home1 = 0.96;
    double home2 = 0.03;
    double home3 = 0.175;
    double score1 = 0.66;
    double score2 = 0.33;
    double score3 = 0.475;
    boolean capacityChecked = false;
    boolean shootingFinished = false;
    boolean motifChecked = false;
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    int flickCounter = 0;
    Servo hood;
    Limelight3A limelight;
    int tagID;
    int patternIndex = 0;
    int greenIndex = 0;
    int purpleIndex = 0;
    boolean leaveActivated = false;
    ArrayList<Flicker> flickOrder = new ArrayList<>();
    ArrayList<Flicker> purple = new ArrayList<>();
    ArrayList<Flicker> green = new ArrayList<>();
    color_sensor_hardware cSensors = new color_sensor_hardware();
    private final Pose startPose = new Pose(81.25, 134.5, Math.toRadians(90)).mirror(); // Start Pose of our robot.
    private final Pose scorePreloadPose = new Pose(94, 104, Math.toRadians(90)).mirror();// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose = new Pose (94,104,Math.toRadians(0)).mirror();
    private final Pose go1Pose = new Pose(93, 90, Math.toRadians(0)).mirror();
    private final Pose pickup1Pose = new Pose(123, 87, Math.toRadians(0)).mirror();
    private final Pose gatePose = new Pose(123.5 , 78.5, Math.toRadians(0)).mirror();
    private final Pose go2Pose = new Pose(93, 61, Math.toRadians(0)).mirror();
    private final Pose pickup2Pose = new Pose(131, 61, Math.toRadians(0)).mirror();
    private final Pose exit2Pose = new Pose (110, 61, Math.toRadians(0)).mirror();
    private final Pose go3Pose = new Pose(93, 38.8, Math.toRadians(0)).mirror();
    private final Pose pickup3Pose = new Pose(131, 38.8, Math.toRadians(0)).mirror();
    private final Pose score3Pose = new Pose(83,112,Math.toRadians(0)).mirror();
    private final Pose leavePose = new Pose (113, 72, Math.toRadians(0)).mirror();
    private PathChain scorePreload, gotoPickup1, grabPickup1, gotoPickup2, grabPickup2, gotoPickup3, grabPickup3, scorePickup1, scorePickup2, scorePickup3, leave, openGate1, openGate2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
                .build();
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        gotoPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePreloadPose, go1Pose))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), go1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(go1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(go1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        openGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, new Pose (106.5,76).mirror(), gatePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(),gatePose.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        gotoPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, go2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), go2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(go2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(go2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        openGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(115, 57.5).mirror(), new Pose(115.5, 76).mirror(), gatePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(),gatePose.getHeading())
                .build();

//        exit2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, exit2Pose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), exit2Pose.getHeading())
//                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(exit2Pose, scorePose))
                .setLinearHeadingInterpolation(exit2Pose.getHeading(), scorePose.getHeading())
                .build();

        gotoPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, go3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), go3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(go3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(go3Pose.getHeading(), pickup3Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, score3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(score3Pose, leavePose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(),leavePose.getHeading())
                .build();

    }
    private RunCommand spinShooter(double targetVel) {
        return new RunCommand(() -> {
            double power = PIDControlShooter(targetVel, shooterTop.getVelocity(AngleUnit.DEGREES),0.0325,0.00325);
            shooterTop.setPower(power);
            shooterBottom.setPower(power);
        });
    }
    private InstantCommand stopShooter() {
        return new InstantCommand(() -> {
            shooterTop.setPower(0);
            shooterBottom.setPower(0);
        });
    }
    private RunCommand moveTurret(double targetDegrees) {
        return new RunCommand(() -> {
            double power = PIDControlTurret(targetDegrees, (double) (turretEncoder.getCurrentPosition() * 360) / (4 * 8192),0.06);
            turret.setPower(power);
        });
    }
    private InstantCommand moveHood (double hoodPos) {
        return new InstantCommand(()->hood.setPosition(hoodPos));
    }
    private RunCommand scoreArtifacts(double nextTime, IntSupplier motif) {
        return new RunCommand(() -> {
            int motifID = motif.getAsInt();
            Flicker f1 = new Flicker(flicker1, home1, score1);
            Flicker f2 = new Flicker(flicker2, home2, score2);
            Flicker f3 = new Flicker(flicker3, home3, score3);
            Enum<color_sensor_hardware.DetectedColor> color1 = cSensors.get1FinalColor();
            Enum<color_sensor_hardware.DetectedColor> color2 = cSensors.get2FinalColor();
            Enum<color_sensor_hardware.DetectedColor> color3 = cSensors.get3FinalColor();

            color_sensor_hardware.DetectedColor[] motifGPP = {
                    color_sensor_hardware.DetectedColor.GREEN,
                    color_sensor_hardware.DetectedColor.PURPLE,
                    color_sensor_hardware.DetectedColor.PURPLE
            };
            color_sensor_hardware.DetectedColor[] motifPGP = {
                    color_sensor_hardware.DetectedColor.PURPLE,
                    color_sensor_hardware.DetectedColor.GREEN,
                    color_sensor_hardware.DetectedColor.PURPLE
            };
            color_sensor_hardware.DetectedColor[] motifPPG = {
                    color_sensor_hardware.DetectedColor.PURPLE,
                    color_sensor_hardware.DetectedColor.PURPLE,
                    color_sensor_hardware.DetectedColor.GREEN
            };
            if (!capacityChecked) {
                purple.clear();
                green.clear();
                if (color1 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f1);
                if (color1 == color_sensor_hardware.DetectedColor.GREEN) green.add(f1);
                if (color2 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f2);
                if (color2 == color_sensor_hardware.DetectedColor.GREEN) green.add(f2);
                if (color3 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f3);
                if (color3 == color_sensor_hardware.DetectedColor.GREEN) green.add(f3);
                patternIndex = 0;
                greenIndex = 0;
                purpleIndex = 0;
                flickerTimer.reset();
                nextTimer.reset();
                shootingFinished = false;
                capacityChecked = true;
            }
            if (motifID == 21) {
                if (patternIndex < motifGPP.length) {

                    color_sensor_hardware.DetectedColor wanted = motifGPP[patternIndex];
                    Flicker current = null;

                    // primary
                    if (wanted == color_sensor_hardware.DetectedColor.GREEN && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    // fallback
                    else if (wanted == color_sensor_hardware.DetectedColor.GREEN && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    // nothing left
                    else {
                        shootingFinished = true;
                    }

                    if (!shootingFinished && current != null) {

                        runFlicker(current);

                        // 🔑 SINGLE ADVANCE POINT
                        if (nextTimer.seconds() >= nextTime) {

                            flickerTimer.reset();
                            nextTimer.reset();

                            // consume artifact
                            if (green.contains(current)) greenIndex++;
                            else purpleIndex++;

                            patternIndex++;
                        }
                    }

                } else {
                    shootingFinished = true;
                    capacityChecked = false;
                    green.clear();
                    purple.clear();
                    greenIndex = 0;
                    purpleIndex = 0;

                    flickerTimer.reset();
                    nextTimer.reset();
                }


            }
            if (motifID == 22) {
                if (patternIndex < motifPGP.length) {

                    color_sensor_hardware.DetectedColor wanted = motifPGP[patternIndex];
                    Flicker current = null;
                    if (green.isEmpty() && purple.isEmpty()){
                        shootingFinished = true;
                    }
                    // primary
                    if (wanted == color_sensor_hardware.DetectedColor.GREEN && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    // fallback
                    else if (wanted == color_sensor_hardware.DetectedColor.GREEN && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    // nothing left
                    else {
                        shootingFinished = true;
                    }

                    if (!shootingFinished && current != null) {

                        runFlicker(current);

                        // 🔑 SINGLE ADVANCE POINT
                        if (nextTimer.seconds() >= nextTime) {

                            flickerTimer.reset();
                            nextTimer.reset();

                            // consume artifact
                            if (green.contains(current)) greenIndex++;
                            else purpleIndex++;

                            patternIndex++;
                        }
                    }

                } else {
                    shootingFinished = true;
                    capacityChecked = false;
                    green.clear();
                    purple.clear();
                    greenIndex = 0;
                    purpleIndex = 0;

                    flickerTimer.reset();
                    nextTimer.reset();
                }

            }
            if (motifID == 23) {
                if (patternIndex < motifPPG.length) {

                    color_sensor_hardware.DetectedColor wanted = motifPPG[patternIndex];
                    Flicker current = null;
                    if (green.isEmpty() && purple.isEmpty()){
                        shootingFinished = true;
                    }

                    // primary
                    if (wanted == color_sensor_hardware.DetectedColor.GREEN && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    // fallback
                    else if (wanted == color_sensor_hardware.DetectedColor.GREEN && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    // nothing left
                    else {
                        shootingFinished = true;
                    }

                    if (!shootingFinished && current != null) {

                        runFlicker(current);

                        // 🔑 SINGLE ADVANCE POINT
                        if (nextTimer.seconds() >= nextTime) {

                            flickerTimer.reset();
                            nextTimer.reset();

                            // consume artifact
                            if (green.contains(current)) greenIndex++;
                            else purpleIndex++;

                            patternIndex++;
                        }
                    }

                } else {
                    shootingFinished = true;
                    capacityChecked = false;
                    green.clear();
                    purple.clear();
                    greenIndex = 0;
                    purpleIndex = 0;

                    flickerTimer.reset();
                    nextTimer.reset();
                }
            }
            if (motifID != 21 && motifID != 22 && motifID != 23){
                if (patternIndex < motifGPP.length) {

                    color_sensor_hardware.DetectedColor wanted = motifGPP[patternIndex];
                    Flicker current = null;
                    if (green.isEmpty() && purple.isEmpty()){
                        shootingFinished = true;
                    }

                    // primary
                    if (wanted == color_sensor_hardware.DetectedColor.GREEN && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    // fallback
                    else if (wanted == color_sensor_hardware.DetectedColor.GREEN && purpleIndex < purple.size()) {
                        current = purple.get(purpleIndex);
                    }
                    else if (wanted == color_sensor_hardware.DetectedColor.PURPLE && greenIndex < green.size()) {
                        current = green.get(greenIndex);
                    }
                    // nothing left
                    else {
                        shootingFinished = true;
                    }

                    if (!shootingFinished && current != null) {

                        runFlicker(current);

                        // 🔑 SINGLE ADVANCE POINT
                        if (nextTimer.seconds() >= nextTime) {

                            flickerTimer.reset();
                            nextTimer.reset();

                            // consume artifact
                            if (green.contains(current)) greenIndex++;
                            else purpleIndex++;

                            patternIndex++;
                        }
                    }

                } else {
                    shootingFinished = true;
                    capacityChecked = false;
                    green.clear();
                    purple.clear();
                    greenIndex = 0;
                    purpleIndex = 0;

                    flickerTimer.reset();
                    nextTimer.reset();
                }
            }
        });
    }
    private void runFlicker(Flicker f) {
        if (flickerTimer.seconds() <= 0.3) {
            f.goScore();
        } else {
            f.goHome();
        }
    }
    private InstantCommand resetFlickers() {
        return new InstantCommand(() -> {
            shootingFinished = false;
            capacityChecked = false;
            flicker1.setPosition(home1);
            flicker2.setPosition(home2);
            flicker3.setPosition(home3);
            flickerTimer.reset();
            nextTimer.reset();
        });
    }
    private InstantCommand stopTurret() {
        return new InstantCommand(() -> turret.setPower(0));
    }

    private InstantCommand spinIntake() {
        return new InstantCommand(() -> {
            Enum<color_sensor_hardware.DetectedColor> color1 = cSensors.get1FinalColor();
            Enum<color_sensor_hardware.DetectedColor> color2 = cSensors.get2FinalColor();
            Enum<color_sensor_hardware.DetectedColor> color3 = cSensors.get3FinalColor();
            boolean detect1 = (color1 != color_sensor_hardware.DetectedColor.UNKNOWN);
            boolean detect2 = (color2 != color_sensor_hardware.DetectedColor.UNKNOWN);
            boolean detect3 = (color3 != color_sensor_hardware.DetectedColor.UNKNOWN);
            boolean isFull = (detect1 && detect2 && detect3);
            if (isFull) intake.setPower(-1);
            else intake.setPower(1);
        });
    }
    private InstantCommand reverseIntake(){
        return new InstantCommand(()-> intake.setPower(-1));
    }
    private InstantCommand stopIntake() {
        return new InstantCommand(() -> intake.setPower(0));
    }
    private RunCommand checkMotif(){
        return new RunCommand(() -> {
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                if (!fiducials.isEmpty()&&!motifChecked) {
                    tagID = fiducials.get(0).getFiducialId();
                    motifChecked = true;
                }
            }
            else telemetry.addData("target", "INVALID");
        });
    }
    private InstantCommand activateLeave(){
        return new InstantCommand(()->{
            leaveActivated = true;
        });
    }
    private double PIDControlShooter(double reference, double state, double Kp, double Kf){
        double error = reference - state;

        return (error * Kp) + (reference * Kf);
    }
    private double PIDControlTurret(double reference, double state, double Kp) {
        double error = reference - state;
        double minimum = 0;
        double maximum = 1;
        double output = (error * Kp);
        if (Math.abs(output) < minimum) {
            output = 0;
        }
        if (output > maximum) output = maximum;
        if (output < -maximum) output = -maximum;
        return output;
    }

    @Override
    public void initialize() {
        super.reset();

        shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
        shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        turretEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        turretEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret = hardwareMap.get(CRServo.class, "turret");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");
        cSensors.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(1);
        limelight.start();
        hood = hardwareMap.get(Servo.class, "hood");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                moveHood(0.7),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        moveTurret(10).withTimeout(1000),
                                        checkMotif().withTimeout(1000)
                                ),
                                moveTurret(51).interruptOn(()-> shootingFinished)
                        ),
                        new FollowPathCommand(follower, scorePreload),
                        spinShooter(135).interruptOn(() -> shootingFinished),
                        new SequentialCommandGroup(
                                new WaitCommand(1500),
                                scoreArtifacts(0.5, ()->tagID).interruptOn(() -> shootingFinished)
                        )
                ),
                resetFlickers(),
                stopShooter(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, gotoPickup1),
                                spinIntake(),
                                new FollowPathCommand(follower, grabPickup1).setGlobalMaxPower(0.1),
                                new WaitCommand(750),
                                stopIntake(),
                                new FollowPathCommand(follower, openGate1).setGlobalMaxPower(0.1),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, scorePickup1).setGlobalMaxPower(1),
                                        new SequentialCommandGroup(
                                                reverseIntake(),
                                                new WaitCommand(750),
                                                stopIntake()
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                spinShooter(135).interruptOn(() -> shootingFinished)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1500),
                                                scoreArtifacts(0.5, ()-> tagID).interruptOn(() -> shootingFinished)
                                        )
                                ),
                                resetFlickers(),
                                stopShooter(),
                                new FollowPathCommand(follower, gotoPickup2),
                                spinIntake(),
                                new FollowPathCommand(follower, grabPickup2).setGlobalMaxPower(0.3),
                                new WaitCommand(750),
                                stopIntake(),
                                new FollowPathCommand(follower, openGate2).setGlobalMaxPower(1),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, scorePickup2).setGlobalMaxPower(1),
                                        new SequentialCommandGroup(
                                                reverseIntake(),
                                                new WaitCommand(750),
                                                stopIntake()
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                spinShooter(135).interruptOn(() -> shootingFinished)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1750),
                                                scoreArtifacts(0.5, ()-> tagID).interruptOn(() -> shootingFinished)
                                        )
                                ),
                                resetFlickers(),
                                stopShooter(),
                                new FollowPathCommand(follower, gotoPickup3),
                                spinIntake(),
                                new FollowPathCommand(follower, grabPickup3).setGlobalMaxPower(0.3),
                                new WaitCommand(750),
                                stopIntake(),
                                activateLeave()
                        ),
                        moveTurret(36).interruptOn(()-> leaveActivated)
                ),
                new ParallelCommandGroup(
                        moveTurret(20.7).interruptOn(()->shootingFinished),
                        new FollowPathCommand(follower, scorePickup3).setGlobalMaxPower(1),
                        new SequentialCommandGroup(
                                reverseIntake(),
                                new WaitCommand(750),
                                stopIntake()
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1500),
                                spinShooter(135).interruptOn(() -> shootingFinished)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(2750),
                                scoreArtifacts(0.4, ()-> tagID).interruptOn(() -> shootingFinished)
                        )
                ),
                resetFlickers(),
                stopShooter(),
                new ParallelCommandGroup(
                        moveTurret(0).withTimeout(10000)
                )
        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        follower.update();
        super.run();
        telemetry.addData("green size", green.size());
        telemetry.addData("purple size", purple.size());
        telemetry.addData("shooterVel", shooterTop.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shootingFinished", shootingFinished);
        telemetry.addData("flickCounter", flickCounter);
        telemetry.addData("capacity checked", capacityChecked);
        telemetry.addData("turretPos", (double) (turretEncoder.getCurrentPosition() * 360) / (4 * 8192));
        telemetry.addData("TAG ID", tagID);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    static class Flicker {
        Servo servo;
        double home, score;
        Flicker(Servo servo, double home, double score){
            this.servo = servo;
            this.home = home;
            this.score = score;
        }
        void goHome(){
            servo.setPosition(home);
        }
        void goScore(){
            servo.setPosition(score);
        }
    }
}
