package org.wheelerschool.robotics.competitionbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.wheelerschool.robotics.library.util.DcMotorUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * Configuration values for the competition bot.
 *
 * @author luciengaitskell
 * @since 161216
 */

public class CompetitionBotConfig {
    private CompetitionBotConfig() {}

    public static List<DcMotor> getLeftMotors(HardwareMap hardwareMap) {
        List<DcMotor> motors = new ArrayList<>();
        motors.add(hardwareMap.dcMotor.get("backLeft"));
        DcMotorUtil.setMotorsDirection(motors, DcMotorSimple.Direction.REVERSE);
        return motors;
    }

    public static List<DcMotor> getRightMotors(HardwareMap hardwareMap) {
        List<DcMotor> motors = new ArrayList<>();
        motors.add(hardwareMap.dcMotor.get("backRight"));
        return motors;
    }
}
