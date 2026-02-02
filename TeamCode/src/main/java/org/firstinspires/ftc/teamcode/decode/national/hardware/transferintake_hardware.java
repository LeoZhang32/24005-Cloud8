package org.firstinspires.ftc.teamcode.decode.national.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.national.teleop_test1;

import java.util.ArrayList;

public class transferintake_hardware {
    private LinearOpMode myOpMode = null;
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    Servo light1;
    Servo light2;
    DcMotorEx intake;
    Boolean capacityChecked = false;
    Boolean shootingFinished = false;
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    int flickCounter = 1;
    double home1 = 0.96;
    double home2 = 0.03;
    double home3 = 0.175;
    double score1 = 0.66;
    double score2 = 0.33;
    double score3 = 0.475;
    double nextTime = 0.4;
    double homeTime = 0.25;
    ArrayList<Flicker> flickOrder = new ArrayList<>();
    color_sensor_hardware cSensors = new color_sensor_hardware();
    public void init(HardwareMap hwMap){
        flicker1 = hwMap.servo.get("flicker1");
        flicker2 = hwMap.servo.get("flicker2");
        flicker3 = hwMap.servo.get("flicker3");
        light1 = hwMap.servo.get("light1");
        light2 = hwMap.servo.get("light2");
        intake = hwMap.get(DcMotorEx.class, "intake");
        cSensors.init(hwMap);
    }


    public void transferandintake(){
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
        if (myOpMode.gamepad2.a){
            nextTime = 0.4;
            homeTime = 0.25;
        }
        else if (myOpMode.gamepad2.b){
            nextTime = 1.2;
            homeTime = 0.45;
        }
        if (myOpMode.gamepad2.a || myOpMode.gamepad2.b){
            if (!flickOrder.isEmpty() && !shootingFinished){
                //actually move the flickers.
                if (flickerTimer.seconds() <= homeTime){
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
                }
            }
        }
        else {
            capacityChecked = false;
            flickerTimer.reset();
            nextTimer.reset();
            flickOrder.clear();
            flickCounter = 1;
            flicker1.setPosition(home1);
            flicker2.setPosition(home2);
            flicker3.setPosition(home3);
        }
        if (myOpMode.gamepad1.a){
            if (detect1 && detect2 && detect3) intake.setPower(-1);
            else intake.setPower(1);
        }
        else if (myOpMode.gamepad1.b){
            intake.setPower(-1);
        }
        else intake.setPower(0);

        if (detect1 && detect2 && detect3){
            light1.setPosition(1);
            light2.setPosition(1);
        }
        else if ((detect1 && detect2 && !detect3) || (detect1 && !detect2 && detect3) || (!detect1 && detect2 && detect3)){
            light1.setPosition(0.66);
            light2.setPosition(0.66);
        }
        else if ((detect1 && !detect2 && !detect3) || (!detect1 && !detect2 && detect3) || (!detect1 && detect2 && !detect3)){
            light1.setPosition(0.33);
            light2.setPosition(0.33);
        }
        else {
            light1.setPosition(0);
            light2.setPosition(0);
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
    public transferintake_hardware(LinearOpMode opmode) { myOpMode = opmode; }

}
