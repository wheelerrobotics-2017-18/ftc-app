package org.wheelerschool.robotics.library.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

/**
 * Helpful functions for "DcMotor"s.
 *
 * @see com.qualcomm.robotcore.hardware.DcMotor
 *
 * @author luciengaitskell
 * @version 1.0
 * @since 161127
 */

public class DcMotorUtil {
    private DcMotorUtil() {}  // Prevent instantiation

    public static void setMotorsRunMode(List<DcMotor> motors, DcMotor.RunMode runMode) {
        /**
         * Set the RunMode of all motors in a list.
         */
        for (DcMotor mtr : motors) {
            mtr.setMode(runMode);
        }
    }

    public static void setMotorsDirection(List<DcMotor> motors, DcMotorSimple.Direction direction) {
        /**
         * Set the Direction of all motors in a list.
         */
        for (DcMotor mtr : motors) {
            mtr.setDirection(direction);
        }
    }

    public static void setMotorsPower(List<DcMotor> motors, double power) {
        /**
         * Set the power of all motors in a list.
         */
        for (DcMotor mtr : motors) {
            mtr.setPower(power);
        }
    }

    public static Long getMotorsPosition(List<DcMotor> motors) {
        /**
         * Get the average encoder position of all motors in a list.
         *
         * Returns:
         *      Long: Average encoder positions of the 'DcMotor's.
         *      null: If there are no motors in the list.
         */
        long encoderTotal = 0;
        int motorCount = 0;

        for (DcMotor mtr : motors) {
            motorCount++;
            encoderTotal += mtr.getCurrentPosition();
        }

        if (motorCount < 1) {
            return null;
        }

        return encoderTotal / motorCount;
    }
}
