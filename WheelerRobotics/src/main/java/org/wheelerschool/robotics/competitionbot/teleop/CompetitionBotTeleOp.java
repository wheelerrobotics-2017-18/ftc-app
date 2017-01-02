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
    private double driveMotorGain = -0.7;

    // Launcher Motors:
    private JoystickButtonUpdated launcherActivateButton;

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
        launcherActivateButton = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad2.x;
            }
        }, false);

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
        JoystickButtonUpdated.JoystickButtonData robotDirectionReverseButtonData =
                robotDirectionReveseButton.getValueIgnoreException();
        if (robotDirectionReverseButtonData.isButtonStateNew && robotDirectionReverseButtonData.buttonState) {
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
        /**
         * The "gamepad2.x" button is used to toggle the launcher on and off. After the launcher has
         *  been activated and "gamepad2.a" is held, the power is set to launching speed. If the
         *  button is let go of, it will return to idling power.
         */
        //  Get current button data:
        JoystickButtonUpdated.JoystickButtonData launcherActivateButtonData =
                launcherActivateButton.getValueIgnoreException();

        double launcherPower;
        // If the launcher is activated:
        if (launcherActivateButtonData.flipStateValue) {
            // If "gamepad2.a" is held down:
            if (gamepad2.a) {
                // Set the launcher to launch mode:
                launcherPower = robot.setLauncherState(CompetitionBotConfig.LauncherMotorsState.LAUNCH);
            } else {
                // Set the launcher to idle mode:
                launcherPower = robot.setLauncherState(CompetitionBotConfig.LauncherMotorsState.IDLE);
            }
        } else {
            // Disable the launcher:
            launcherPower = robot.setLauncherState(CompetitionBotConfig.LauncherMotorsState.DISABLE);
        }

        telemetry.addData("Launcher Motor Speed", launcherPower);

        // Loader Control:
        double feederSpeed = gamepad2.right_trigger * feederServoGain;
        robot.feederServo.setPower(feederSpeed);
        telemetry.addData("Feeder Servo Speed", feederSpeed);
    }
}
