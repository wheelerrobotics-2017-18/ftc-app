package org.wheelerschool.robotics.competitionbot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.wheelerschool.robotics.competitionbot.CompetitionBotConfig;
import org.wheelerschool.robotics.library.util.DcMotorUtil;
import org.wheelerschool.robotics.library.util.joystick.JoystickButtonUpdated;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;

import static org.wheelerschool.robotics.competitionbot.CompetitionBotConfig.defaultRobotForwards;

/**
 * An OpMode which allows for full control of the competition bot.
 *
 * @author luciengaitskell
 * @since  161219
 */

@TeleOp
public class CompetitionBotTeleOp extends OpMode {
    private String LOG_TAG = "Comp Bot TeleOp";

    CompetitionBotConfig robot;
    JoystickButtonUpdated robotDirectionReveseButton;
    boolean robotForward = defaultRobotForwards;

    // Beacon Pushers:

    // Collector Motors:
    private double collectorMotorsSpeed = -0.5;
    private List<DcMotor> collectorMotors;

    // Drive Motors:
    private double driveMotorGain = -1;

    // Launcher Motors:
    private double launcherMotorGain = -0.8;

    // Feeder Servo:
    private double feederServoGain = 1;


    @Override
    public void init() {
        robot = new CompetitionBotConfig(hardwareMap);
        robotDirectionReveseButton = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad1.y;
            }
        });

        // Beacon Pushers:

        // Collector Motors:
        collectorMotors = new ArrayList<>();
        collectorMotors.add(hardwareMap.dcMotor.get("collector"));
    }

    @Override
    public void loop() {
        // Collector Motors:
        if (gamepad1.right_bumper) {
            DcMotorUtil.setMotorsPower(collectorMotors, collectorMotorsSpeed);
        } else {
            DcMotorUtil.setMotorsPower(collectorMotors, 0);
        }

        // Beacon Push:
        double beaconTriggerGain = 1;
        robot.pusherLeft.servo.setPosition(gamepad1.left_trigger * beaconTriggerGain);
        robot.pusherRight.servo.setPosition(gamepad1.right_trigger * beaconTriggerGain);

        // Drive Direction Set:
        JoystickButtonUpdated.JoystickButtonData robotDirectionReveseButtonData =
                robotDirectionReveseButton.getValueIgnoreException();
        if (robotDirectionReveseButtonData.isButtonStateNew && robotDirectionReveseButtonData.buttonState) {
            robotForward = !robotForward;
            robot.setRobotDirection(robotForward);
        }

        telemetry.addData("Robot Forward", robotForward);

        // Drive Motors:
        double leftSpeed = gamepad1.left_stick_y * driveMotorGain;
        telemetry.addData("Left Speed", leftSpeed);
        DcMotorUtil.setMotorsPower(robot.leftMotors, leftSpeed);
        double rightSpeed = gamepad1.right_stick_y * driveMotorGain;
        telemetry.addData("Right Speed", rightSpeed);
        DcMotorUtil.setMotorsPower(robot.rightMotors, rightSpeed);

        telemetry.addData("Left Motors Encoder", DcMotorUtil.getMotorsPosition(robot.leftMotors));
        telemetry.addData("Right Motors Encoder", DcMotorUtil.getMotorsPosition(robot.rightMotors));

        // Launcher Control:
        double launcherSpeed = gamepad2.left_stick_y * launcherMotorGain;
        DcMotorUtil.setMotorsPower(robot.launcherMotors, launcherSpeed);
        telemetry.addData("Launcher Motor Speed", launcherSpeed);

        // Loader Control:
        double feederSpeed = gamepad2.right_trigger * feederServoGain;
        robot.feederServo.setPower(feederSpeed);
        telemetry.addData("Feeder Servo Speed", feederSpeed);
    }
}
