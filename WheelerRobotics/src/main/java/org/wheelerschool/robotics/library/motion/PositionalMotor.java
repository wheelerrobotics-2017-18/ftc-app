package org.wheelerschool.robotics.library.motion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by luciengaitskell on 1/3/18.
 */

public class PositionalMotor {
    /**
     * A class allowing for easy control of DcMotors with encoders, to preset encoder positions.
     */
    public DcMotor dcMotor;
    public int currentIdx;

    private int[] positions;

    private boolean wasManual = false;


    public PositionalMotor(DcMotor m, int[] pos) {
        this(m, pos, 0);
    }

    public PositionalMotor(DcMotor m, int[] pos, int startingIdx) {
        dcMotor = m;
        positions = pos;
        currentIdx = startingIdx;
    }

    public void stop() {
        dcMotor.setPower(0);

        if (motorActive()) {
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private int updateIdx(int idx) {
        currentIdx = Range.clip(idx, 0, positions.length-1);
        return currentIdx;
    }

    public void moveTo(int idx, double power) {
        moveToEnc(positions[updateIdx(idx)], power);
    }

    public void moveToEnc(int enc, double power) {
        dcMotor.setPower(0);
        dcMotor.setTargetPosition(enc);
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotor.setPower(power);
    }

    public void moveRel(int idxRel, double power) {
        moveTo(currentIdx + idxRel, power);
    }

    public boolean motorActive() {
        return dcMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION;
    }

    public boolean manualOverride(float ctl) {
        return manualOverride(ctl, true);
    }
    public boolean manualOverride(float ctl, boolean enable) {
        if (ctl != 0 && enable) {
            if (motorActive()) {
                stop();
            }

            wasManual = true;
            dcMotor.setPower(ctl);

            return true;
        } else {
            if (wasManual) {
                stop();
                wasManual = false;
            }
            return false;
        }
    }
}
