package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp
public class colortest2 extends LinearOpMode {
    color_sensor_hardware cSensors = new color_sensor_hardware();
    Boolean detect1;
    Boolean detect2;
    Boolean detect3;
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();
    double loopTime = 0;
    ArrayList<Flicker> flickOrder = new ArrayList<>();
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    int flickCounter = 1;
    double home1 = 0.96;
    double home2 = 0.03;
    double home3 = 0.175;
    double score1 = 0.66;
    double score2 = 0.33;
    double score3 = 0.475;
    double nextTime = 0.4;
    double homeTime = 0.25;

    boolean shootingFinished = false;
    Boolean capacityChecked = false;
    @Override
    public void runOpMode() throws InterruptedException {
        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");
        cSensors.init(hardwareMap);
        nextTimer.reset();

        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            loopTimer.reset();
            if (gamepad1.a){
                //if haven't checked for artifact this press, check
                if (!capacityChecked){
                    //check if each spot has artifact
                    detect1 = cSensors.checkDetected1();
                    detect2 = cSensors.checkDetected2();
                    detect3 = cSensors.checkDetected3();
                    //add detected spots to array to be shot
                    if (detect1){
                        flickOrder.add(new Flicker(flicker1, home1, score1));
                    }
                    if (detect2){
                        flickOrder.add(new Flicker(flicker2, home2, score2));
                    }
                    if (detect3){
                        flickOrder.add(new Flicker(flicker3, home3, score3));
                    }
                    //we have detected for artifacts! for next loops in press, don't check again
                    capacityChecked = true;
                    shootingFinished = false;
                }
                if (!flickOrder.isEmpty() && !shootingFinished){
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
                    }

                    //actually move the flickers.
                    if (flickerTimer.seconds() <= homeTime){
                        //if the PIDtimer is before time to move back, it's in score position.
                        flickOrder.get(flickCounter - 1).goScore();
                    }
                    //if PIDtimer is after time to move back, move back.
                    else flickOrder.get(flickCounter - 1).goHome();
                }
            }
            //once input is let go, be ready to check again, and reset everything.
            else {
                detect1 = cSensors.checkDetected1();
                detect2 = cSensors.checkDetected2();
                detect3 = cSensors.checkDetected3();
                telemetry.addData("1", detect1);
                telemetry.addData("2", detect2);
                telemetry.addData("3", detect3);

                capacityChecked = false;
                flickerTimer.reset();
                nextTimer.reset();
                flickOrder.clear();
                flickCounter = 1;
                flicker1.setPosition(home1);
                flicker2.setPosition(home2);
                flicker3.setPosition(home3);
            }

            loopTime = 1/loopTimer.seconds();
            telemetry.addData("loop time (Hz)", loopTime);
            telemetry.update();
        }
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
