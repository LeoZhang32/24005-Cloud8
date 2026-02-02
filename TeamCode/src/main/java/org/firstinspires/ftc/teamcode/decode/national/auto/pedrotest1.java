package org.firstinspires.ftc.teamcode.decode.national.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

@Autonomous(name = "AAA PEDRO")
public class pedrotest1 extends CommandOpMode {
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
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    int flickCounter = 1;

    ArrayList<pedrotest1.Flicker> flickOrder = new ArrayList<>();
    color_sensor_hardware cSensors = new color_sensor_hardware();
    private final Pose startPose = new Pose(120, 127, Math.toRadians(36)); // Start Pose of our robot.
    private final Pose scorePreloadPose = new Pose(94, 104, Math.toRadians(36));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose = new Pose (94,104,Math.toRadians(0));
    private final Pose go1Pose = new Pose(93, 84, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(120, 83.86, Math.toRadians(0));
    private final Pose go2Pose = new Pose(93, 59.4, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(120, 59.4, Math.toRadians(0));
    private final Pose go3Pose = new Pose(93, 35.8, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(120, 35.8, Math.toRadians(0));
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private PathChain scorePreload, gotoPickup1, grabPickup1, gotoPickup2, grabPickup2, gotoPickup3, grabPickup3, scorePickup1, scorePickup2, scorePickup3, leave;

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
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
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
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
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
            double power = PIDControlTurret(targetDegrees, ((double) (-turretEncoder.getCurrentPosition() * 360) / (4 * 8192)),0.06);
            turret.setPower(power);
        });
    }
    private RunCommand scoreArtifacts(double nextTime) {
        return new RunCommand(() -> {
            boolean detect1 = false;
            boolean detect2 = false;
            boolean detect3 = false;

            if (!capacityChecked){
                detect1 = cSensors.checkDetected1();
                detect2 = cSensors.checkDetected2();
                detect3 = cSensors.checkDetected3();
                //check if each spot has artifact
                //add detected spots to array to be shot
                if (detect1){
                    flickOrder.add(new pedrotest1.Flicker(flicker1, home1, score1));
                }
                if (detect2){
                    flickOrder.add(new pedrotest1.Flicker(flicker2, home2, score2));
                }
                if (detect3){
                    flickOrder.add(new pedrotest1.Flicker(flicker3, home3, score3));
                }
                //we have detected for artifacts! for next loops in press, don't check again
                capacityChecked = true;
                shootingFinished = false;
                nextTimer.reset();
                flickerTimer.reset();
            }
            if (!flickOrder.isEmpty() && !shootingFinished){
                //actually move the flickers.
                if (flickerTimer.seconds() <= 0.25){
                    //if the timer is before time to move back, it's in score position.
                    flickOrder.get(flickCounter - 1).goScore();
                }
                //if timer is after time to move back, move back.
                else flickOrder.get(flickCounter - 1).goHome();

                //if we reach the time to cycle to the next artifact, plus 1 to the counter and reset timers.
                if (nextTimer.seconds() >= nextTime){
                    flickerTimer.reset();
                    nextTimer.reset();
                    flickCounter += 1;
                }
                //once the counter reaches larger than the number of spots in the array.
                //this means that all artifacts have been shot.
                if (flickCounter > flickOrder.size()){
                    flickCounter = 1;
                    flickOrder.clear();
                    shootingFinished = true;
                    capacityChecked = false;
                }
            }
            else{
                flicker1.setPosition(home1);
                flicker2.setPosition(home2);
                flicker3.setPosition(home3);
            }
        });
    }
    private InstantCommand resetFlickers() {
        return new InstantCommand(() -> {
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
        return new InstantCommand(() -> intake.setPower(1));
    }
    private InstantCommand stopIntake() {
        return new InstantCommand(() -> intake.setPower(0));
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

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, scorePreload),
                        moveTurret(0).withTimeout(4500),
                        spinShooter(140).withTimeout(4500),
                        new SequentialCommandGroup(
                                new WaitCommand(2500),
                                scoreArtifacts(0.4).withTimeout(2000)
                        )
                ),
                resetFlickers(),
                stopTurret(),
                stopShooter(),
                new FollowPathCommand(follower, gotoPickup1),
                spinIntake(),
                new FollowPathCommand(follower, grabPickup1, 0.5),
                new WaitCommand(500),
                stopIntake(),
                new FollowPathCommand(follower,scorePickup1),
                new WaitCommand(1000),
                new FollowPathCommand(follower, gotoPickup2),
                spinIntake(),
                new FollowPathCommand(follower, grabPickup2, 0.5),
                new WaitCommand(500),
                stopIntake(),
                new FollowPathCommand(follower,scorePickup2),
                new WaitCommand(1000),
                new FollowPathCommand(follower, gotoPickup3),
                spinIntake(),
                new FollowPathCommand(follower, grabPickup3, 0.5),
                new WaitCommand(500),
                stopIntake(),
                new FollowPathCommand(follower,scorePickup3),
                new WaitCommand(1000)

        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        follower.update();
        super.run();
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
