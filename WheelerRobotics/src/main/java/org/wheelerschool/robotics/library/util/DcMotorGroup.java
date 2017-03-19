package org.wheelerschool.robotics.library.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

/**
 * Created by luciengaitskell on 3/19/17.
 */

public class DcMotorGroup extends ArrayList<DcMotor> {
    public void setMotorsRunMode(DcMotor.RunMode runMode) {
        /**
         * Set the RunMode of the motors.
         * @param runMode: The `DcMotor.RunMode` to set the motors to.
         */
        DcMotorUtil.setMotorsRunMode(this, runMode);
    }

    public void setMotorsDirection(DcMotorSimple.Direction direction) {
        /**
         * Set the Direction of the motors.
         * @param direction: The direction to set the motors to.
         */
        DcMotorUtil.setMotorsDirection(this, direction);
    }

    public void setMotorsPower(double power) {
        /**
         * Set the power of the motors.
         * @param power: The power to set the motors to.
         */
        DcMotorUtil.setMotorsPower(this, power);
    }

    public Long getMotorsPosition() {
        /**
         * Get the average encoder position of the motors.
         * @return the average encoder positioms of the motors. Is 'null' if no motors.
         */
        return DcMotorUtil.getMotorsPosition(this);
    }
}
