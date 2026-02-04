package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class CycleGamepad {
    private final Gamepad gamepad;

    private boolean previousXState = false;
    private boolean previousYState = false;
    private boolean previousAState = false;
    private boolean previousBState = false;
    private boolean previousLBState = false;
    private boolean previousRBState = false;
    private boolean previousStartState = false;
    public int xPressCount, yPressCount, aPressCount, bPressCount, lbPressCount, rbPressCount, startPressCount = 0;

    public CycleGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void updateX(int cycles) {
        boolean currentXState = gamepad.x;
        if (currentXState && !previousXState) { // Prevent "button held down" behavior
            xPressCount++;
            if (xPressCount > (cycles-1)){
                xPressCount = 0;
            }
        }

        previousXState = currentXState;
    }

    public void updateY(int cycles) {
        boolean currentYState = gamepad.y;
        if (currentYState && !previousYState) { // Prevent "button held down" behavior
            // Increment the press count and ensure it loops between 0 and 2
            yPressCount++;
            if (yPressCount > (cycles-1)){
                yPressCount = 0;
            }
        }
        previousYState = currentYState;
    }

    public void updateA(int cycles) {
        boolean currentAState = gamepad.a;
        if (currentAState && !previousAState) { // Prevent "button held down" behavior
            // Increment the press count and ensure it loops between 0 and 2
            aPressCount++;
            if (aPressCount > (cycles-1)){
                aPressCount = 0;
            }
        }
        previousAState = currentAState;
    }

    public void updateB(int cycles) {
        boolean currentBState = gamepad.b;
        if (currentBState && !previousBState) { // Prevent "button held down" behavior
            // Increment the press count and ensure it loops between 0 and 2
            bPressCount++;
            if (bPressCount > (cycles-1)){
                bPressCount = 0;
            }
        }
        previousBState = currentBState;
    }
    public void updateLB(int cycles) {
        boolean currentLBState = gamepad.left_bumper;
        if (currentLBState && !previousLBState) { // Prevent "button held down" behavior
            // Increment the press count and ensure it loops between 0 and 2
            lbPressCount++;
            if (lbPressCount > (cycles-1)){
                lbPressCount = 0;
            }
        }
        previousLBState = currentLBState;
    }
    public void updateRB(int cycles) {
        boolean currentRBState = gamepad.right_bumper;
        if (currentRBState && !previousRBState) { // Prevent "button held down" behavior
            // Increment the press count and ensure it loops between 0 and 2
            rbPressCount++;
            if (rbPressCount > (cycles-1)){
                rbPressCount = 0;
            }
        }
        previousRBState = currentRBState;
    }
    public void updateStart(int cycles) {
        boolean currentStartState = gamepad.start;
        if (currentStartState && !previousStartState) { // Prevent "button held down" behavior
            // Increment the press count and ensure it loops between 0 and 2
            startPressCount++;
            if (startPressCount > (cycles-1)){
                startPressCount = 0;
            }
        }
        previousStartState = currentStartState;
    }
}
