package org.firstinspires.ftc.teamcode.decode.national.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class transferintake_hardware {
    private LinearOpMode myOpMode = null;
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;

    Servo light1;
    Servo light2;
    Servo light3;
    DcMotorEx intake;
    Boolean capacityChecked = false;
    Boolean shootingFinished = false;
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    int flickCounter = 0;
    double home1 = 0.96;
    double home2 = 0.03;
    double home3 = 0.175;
    double score1 = 0.66;
    double score2 = 0.33;
    double score3 = 0.475;
    double nextTime = 0.4;
    double homeTime = 0.25;
    int patternIndex = 0;
    int greenIndex = 0;
    int purpleIndex = 0;
    boolean lastX = false;
    boolean lastY = false;

    ArrayList<Flicker> flickOrder = new ArrayList<>();
    ArrayList<Flicker> purple = new ArrayList<>();
    ArrayList<Flicker> green = new ArrayList<>();

    private void runFlicker(Flicker f) {
        if (flickerTimer.seconds() <= homeTime) {
            f.goScore();
        } else {
            f.goHome();
        }
    }

    color_sensor_hardware cSensors = new color_sensor_hardware();
    public void init(HardwareMap hwMap){
        flicker1 = hwMap.servo.get("flicker1");
        flicker2 = hwMap.servo.get("flicker2");
        flicker3 = hwMap.servo.get("flicker3");
        light1 = hwMap.servo.get("light1");
        light2 = hwMap.servo.get("light2");
        light3 = hwMap.servo.get("light3");
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
            light1.setPosition(0.6);
            light2.setPosition(0.6);
        }
        else if ((detect1 && detect2 && !detect3) || (detect1 && !detect2 && detect3) || (!detect1 && detect2 && detect3)){
            light1.setPosition(0.388);
            light2.setPosition(0.388);
        }
        else if ((detect1 && !detect2 && !detect3) || (!detect1 && !detect2 && detect3) || (!detect1 && detect2 && !detect3)){
            light1.setPosition(0.277);
            light2.setPosition(0.277);
        }
        else {
            light1.setPosition(0);
            light2.setPosition(0);
        }
    }
    public void sortTransferAndIntake(boolean manualMode, int motif) {

        Flicker f1 = new Flicker(flicker1, home1, score1);
        Flicker f2 = new Flicker(flicker2, home2, score2);
        Flicker f3 = new Flicker(flicker3, home3, score3);
        Enum<color_sensor_hardware.DetectedColor> color1 = cSensors.get1FinalColor();
        Enum<color_sensor_hardware.DetectedColor> color2 = cSensors.get2FinalColor();
        Enum<color_sensor_hardware.DetectedColor> color3 = cSensors.get3FinalColor();
        boolean detect1 = (color1 != color_sensor_hardware.DetectedColor.UNKNOWN);
        boolean detect2 = (color2 != color_sensor_hardware.DetectedColor.UNKNOWN);
        boolean detect3 = (color3 != color_sensor_hardware.DetectedColor.UNKNOWN);

        if((myOpMode.gamepad2.a || myOpMode.gamepad2.b) && shootingFinished){
            shootingFinished = false;
            capacityChecked = false;
        }

        if (!manualMode) {
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

            if (myOpMode.gamepad2.a) {
                nextTime = 0.4;
                homeTime = 0.25;
            } else if (myOpMode.gamepad2.b) {
                nextTime = 1.2;
                homeTime = 0.45;
            }
            if (myOpMode.gamepad2.a || myOpMode.gamepad2.b) {
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
                if (motif == 1) {
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
                if (motif == 2) {
                    if (patternIndex < motifPGP.length) {

                        color_sensor_hardware.DetectedColor wanted = motifPGP[patternIndex];
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
                if (motif == 3) {
                    if (patternIndex < motifPPG.length) {

                        color_sensor_hardware.DetectedColor wanted = motifPPG[patternIndex];
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
            }
            else {
                f1.goHome();
                f2.goHome();
                f3.goHome();
            }
        }
        else { // MANUAL MODE

            boolean x = myOpMode.gamepad2.x;
            boolean y = myOpMode.gamepad2.y;

            boolean xPressed = x && !lastX;
            boolean yPressed = y && !lastY;

            // build lists ONCE
            if ((xPressed || yPressed) && !capacityChecked) {
                purple.clear();
                green.clear();

                if (color1 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f1);
                if (color1 == color_sensor_hardware.DetectedColor.GREEN)  green.add(f1);

                if (color2 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f2);
                if (color2 == color_sensor_hardware.DetectedColor.GREEN)  green.add(f2);

                if (color3 == color_sensor_hardware.DetectedColor.PURPLE) purple.add(f3);
                if (color3 == color_sensor_hardware.DetectedColor.GREEN)  green.add(f3);

                purpleIndex = 0;
                greenIndex = 0;

                capacityChecked = true;
                shootingFinished = false;
            }

            // X = PURPLE
            if (xPressed && purpleIndex < purple.size()) {
                Flicker f = purple.get(purpleIndex);

                f.goScore();
                flickerTimer.reset();

                purpleIndex++;
            }

            // Y = GREEN
            if (yPressed && greenIndex < green.size()) {
                Flicker f = green.get(greenIndex);

                f.goScore();
                flickerTimer.reset();

                greenIndex++;
            }

            // return all flickers home after time
            if (flickerTimer.seconds() > homeTime) {
                f1.goHome();
                f2.goHome();
                f3.goHome();
            }

            // done condition
            if (purpleIndex >= purple.size() && greenIndex >= green.size()) {
                shootingFinished = true;
                capacityChecked = false;
            }

            lastX = x;
            lastY = y;
        }

        if (myOpMode.gamepad1.a) {
            if (detect1 && detect2 && detect3) intake.setPower(-1);
            else intake.setPower(1);
        } else if (myOpMode.gamepad1.b) {
            intake.setPower(-1);
        } else intake.setPower(0);

        if (color1 == color_sensor_hardware.DetectedColor.PURPLE) light1.setPosition(0.722);
        if (color2 == color_sensor_hardware.DetectedColor.PURPLE) light2.setPosition(0.722);
        if (color3 == color_sensor_hardware.DetectedColor.PURPLE) light3.setPosition(0.722);
        if (color1 == color_sensor_hardware.DetectedColor.GREEN) light1.setPosition(0.5);
        if (color2 == color_sensor_hardware.DetectedColor.GREEN) light2.setPosition(0.5);
        if (color3 == color_sensor_hardware.DetectedColor.GREEN) light3.setPosition(0.5);
        if (color1 == color_sensor_hardware.DetectedColor.UNKNOWN) light1.setPosition(0);
        if (color2 == color_sensor_hardware.DetectedColor.UNKNOWN) light2.setPosition(0);
        if (color3 == color_sensor_hardware.DetectedColor.UNKNOWN) light3.setPosition(0);

        myOpMode.telemetry.addData("patternIndex", patternIndex);
        myOpMode.telemetry.addData("greenIndex", greenIndex);
        myOpMode.telemetry.addData("purpleIndex", purpleIndex);
        myOpMode.telemetry.addData("green size", green.size());
        myOpMode.telemetry.addData("purple size", purple.size());
        myOpMode.telemetry.addData("shootingFinished", shootingFinished);

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
