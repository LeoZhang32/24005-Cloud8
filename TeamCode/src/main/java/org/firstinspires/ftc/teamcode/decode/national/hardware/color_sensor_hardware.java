package org.firstinspires.ftc.teamcode.decode.national.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;

public class color_sensor_hardware {
    NormalizedColorSensor A1;
    NormalizedColorSensor B1;
    NormalizedColorSensor A2;
    NormalizedColorSensor B2;
    NormalizedColorSensor A3;
    NormalizedColorSensor B3;
    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN,
        DETECTED
    }
    public void init (HardwareMap hwMap){
        A1 = hwMap.get(NormalizedColorSensor.class, "1a");
        B1 = hwMap.get(NormalizedColorSensor.class, "1b");
        A2 = hwMap.get(NormalizedColorSensor.class, "2a");
        B2 = hwMap.get(NormalizedColorSensor.class, "2b");
        A3 = hwMap.get(NormalizedColorSensor.class, "3a");
        B3 = hwMap.get(NormalizedColorSensor.class, "3b");
    }
    public ArrayList<Float> getA1RGB(){
        ArrayList<Float> rgb = new ArrayList<>();
        NormalizedRGBA colors = A1.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        rgb.add(normRed);
        rgb.add(normGreen);
        rgb.add(normBlue);
        return rgb;
    }
    public ArrayList<Float> getA2RGB(){
        ArrayList<Float> rgb = new ArrayList<>();
        NormalizedRGBA colors = A2.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        rgb.add(normRed);
        rgb.add(normGreen);
        rgb.add(normBlue);
        return rgb;
    }
    public ArrayList<Float> getA3RGB(){
        ArrayList<Float> rgb = new ArrayList<>();
        NormalizedRGBA colors = A3.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        rgb.add(normRed);
        rgb.add(normGreen);
        rgb.add(normBlue);
        return rgb;
    }
    public ArrayList<Float> getB1RGB(){
        ArrayList<Float> rgb = new ArrayList<>();
        NormalizedRGBA colors = B1.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        rgb.add(normRed);
        rgb.add(normGreen);
        rgb.add(normBlue);
        return rgb;
    }
    public ArrayList<Float> getB2RGB(){
        ArrayList<Float> rgb = new ArrayList<>();
        NormalizedRGBA colors = B2.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        rgb.add(normRed);
        rgb.add(normGreen);
        rgb.add(normBlue);
        return rgb;
    }
    public ArrayList<Float> getB3RGB(){
        ArrayList<Float> rgb = new ArrayList<>();
        NormalizedRGBA colors = B3.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        rgb.add(normRed);
        rgb.add(normGreen);
        rgb.add(normBlue);
        return rgb;
    }
    private DetectedColor get1ADetectedColors() {
        NormalizedRGBA colors = A1.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red2", normRed);
//        telemetry.addData("green2", normGreen);
//        telemetry.addData("blue2", normBlue);

        if (normGreen<0.32){
            return DetectedColor.PURPLE;
        }
        else if (normGreen>0.49){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
    private DetectedColor get1BDetectedColors() {
        NormalizedRGBA colors = B1.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red2", normRed);
//        telemetry.addData("green2", normGreen);
//        telemetry.addData("blue2", normBlue);

        if (normGreen<0.32){
            return DetectedColor.PURPLE;
        }
        else if (normGreen>0.49){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
    private DetectedColor get2ADetectedColors() {
        NormalizedRGBA colors = A2.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red1", normRed);
//        telemetry.addData("green1", normGreen);
//        telemetry.addData("blue1", normBlue);
        if (normGreen<0.32){
            return DetectedColor.PURPLE;
        }
        else if (normGreen>0.49){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }

    }
    private DetectedColor get2BDetectedColors() {
        NormalizedRGBA colors = B2.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red2", normRed);
//        telemetry.addData("green2", normGreen);
//        telemetry.addData("blue2", normBlue);

        if (normGreen<0.32){
            return DetectedColor.PURPLE;
        }
        else if (normGreen>0.49){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
    private DetectedColor get3ADetectedColors() {
        NormalizedRGBA colors = A3.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red2", normRed);
//        telemetry.addData("green2", normGreen);
//        telemetry.addData("blue2", normBlue);

        if (normGreen<0.32){
            return DetectedColor.PURPLE;
        }
        else if (normGreen>0.48){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
    private DetectedColor get3BDetectedColors() {
        NormalizedRGBA colors = B3.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red2", normRed);
//        telemetry.addData("green2", normGreen);
//        telemetry.addData("blue2", normBlue);

        if (normGreen<0.32){
            return DetectedColor.PURPLE;
        }
        else if (normGreen>0.49){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
    public DetectedColor get1FinalColor() {
        Enum a = get1ADetectedColors();
        Enum b = get1BDetectedColors();
        if (a == color_sensor_hardware.DetectedColor.PURPLE && b == DetectedColor.GREEN){
            return DetectedColor.UNKNOWN;
        }
        if (a == color_sensor_hardware.DetectedColor.PURPLE ||b == color_sensor_hardware.DetectedColor.PURPLE){
            return DetectedColor.PURPLE;
        }
        else if(a == color_sensor_hardware.DetectedColor.GREEN ||b == color_sensor_hardware.DetectedColor.GREEN){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
    public DetectedColor get2FinalColor() {
        Enum a = get2ADetectedColors();
        Enum b = get2BDetectedColors();
        if (a == color_sensor_hardware.DetectedColor.PURPLE && b == DetectedColor.GREEN){
            return DetectedColor.UNKNOWN;
        }
        if (a == color_sensor_hardware.DetectedColor.PURPLE ||b == color_sensor_hardware.DetectedColor.PURPLE){
            return DetectedColor.PURPLE;
        }
        else if(a == color_sensor_hardware.DetectedColor.GREEN ||b == color_sensor_hardware.DetectedColor.GREEN){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
    public DetectedColor get3FinalColor() {
        Enum a = get3ADetectedColors();
        Enum b = get3BDetectedColors();
        if (a == color_sensor_hardware.DetectedColor.PURPLE && b == DetectedColor.GREEN){
            return DetectedColor.UNKNOWN;
        }
        if (a == color_sensor_hardware.DetectedColor.PURPLE ||b == color_sensor_hardware.DetectedColor.PURPLE){
            return DetectedColor.PURPLE;
        }
        else if(a == color_sensor_hardware.DetectedColor.GREEN ||b == color_sensor_hardware.DetectedColor.GREEN){
            return DetectedColor.GREEN;
        }
        else {
            return DetectedColor.UNKNOWN;
        }

    }
    public boolean checkDetected1(){
        return get1FinalColor() != DetectedColor.UNKNOWN;
    }
    public boolean checkDetected2(){
        return get2FinalColor() != DetectedColor.UNKNOWN;
    }
    public boolean checkDetected3(){
        return get3FinalColor() != DetectedColor.UNKNOWN;
    }
}
