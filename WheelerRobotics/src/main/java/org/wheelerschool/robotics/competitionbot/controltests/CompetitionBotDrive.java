package org.wheelerschool.robotics.competitionbot.controltests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.wheelerschool.robotics.competitionbot.CompetitionBotConfig;
import org.wheelerschool.robotics.library.util.DcMotorUtil;

import java.util.List;

/**
 * An OpMode which allows for simple drive control of the competition bot.
 *
 * @author luciengaitskell
 * @since 161219
 */

@TeleOp
public class CompetitionBotDrive extends OpMode {
    private double motorGain = 1;
    private List<DcMotor> leftMotors;
    private List<DcMotor> rightMotors;

    @Override
    public void init() {
        leftMotors = CompetitionBotConfig.getLeftMotors(hardwareMap);
        DcMotorUtil.setMotorsRunMode(leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotors = CompetitionBotConfig.getRightMotors(hardwareMap);
        DcMotorUtil.setMotorsRunMode(rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        DcMotorUtil.setMotorsPower(leftMotors, gamepad1.left_stick_y * motorGain);
        DcMotorUtil.setMotorsPower(rightMotors, -gamepad1.right_stick_y * motorGain);

        telemetry.addData("Left Motors Encoder", DcMotorUtil.getMotorsPosition(leftMotors));
        telemetry.addData("Right Motors Encoder", DcMotorUtil.getMotorsPosition(rightMotors));
    }
}
