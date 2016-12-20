package org.wheelerschool.robotics.competitionbot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.wheelerschool.robotics.competitionbot.CompetitionBotConfig;
import org.wheelerschool.robotics.library.util.DcMotorUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * An OpMode which allows for full control of the competition bot.
 *
 * @author luciengaitskell
 * @since  161219
 */

@TeleOp
public class CompetitionBotTeleOp extends OpMode {
    private String LOG_TAG = "Comp Bot TeleOp";

    // Collector Motors:
    private double collectorMotorsSpeed = -0.5;
    private List<DcMotor> collectorMotors;

    // Drive Motors:
    private double driveMotorGain = 0.5;
    private List<DcMotor> leftMotors;
    private List<DcMotor> rightMotors;


    @Override
    public void init() {
        // Collector Motors:
        collectorMotors = new ArrayList<>();
        collectorMotors.add(hardwareMap.dcMotor.get("collector"));

        // Drive Motors:
        leftMotors = CompetitionBotConfig.getLeftMotors(hardwareMap);
        DcMotorUtil.setMotorsRunMode(leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotors = CompetitionBotConfig.getRightMotors(hardwareMap);
        DcMotorUtil.setMotorsRunMode(rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorUtil.setMotorsDirection(rightMotors, DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Collector Motors:
        if (gamepad1.right_bumper) {
            DcMotorUtil.setMotorsPower(collectorMotors, collectorMotorsSpeed);
        } else {
            DcMotorUtil.setMotorsPower(collectorMotors, 0);
        }


        // Drive Motors:
        DcMotorUtil.setMotorsPower(leftMotors, gamepad1.left_stick_y * driveMotorGain);
        DcMotorUtil.setMotorsPower(rightMotors, gamepad1.right_stick_y * driveMotorGain);

        telemetry.addData("Left Motors Encoder", DcMotorUtil.getMotorsPosition(leftMotors));
        telemetry.addData("Right Motors Encoder", DcMotorUtil.getMotorsPosition(rightMotors));
    }
}
