package org.wheelerschool.robotics.library.motion;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by luciengaitskell on 12/22/17.
 */

public class ServoTwoPos {
    /**
     * Allows for a servo to easily be set to two predefined positions.
     */

    public Servo s;
    private double truePos;
    private double falsePos;

    public ServoTwoPos(Servo s, double truePos, double falsePos) {
        this.s = s;
        this.truePos = truePos;
        this.falsePos = falsePos;
    }

    public void setState(boolean state) {
        if (state) {
            s.setPosition(truePos);
        } else {
            s.setPosition(falsePos);
        }
    }
}
