package org.wheelerschool.robotics.competitionbot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.wheelerschool.robotics.competitionbot.CompetitionBotConfig;
import org.wheelerschool.robotics.library.util.DcMotorUtil;
import org.wheelerschool.robotics.library.util.LinearOpModeActiveCallable;
import org.wheelerschool.robotics.library.util.LinearOpModeUtil;
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
public class CompetitionBotTeleOp extends LinearOpMode {
    public CompetitionBotTeleOp() {
        super();
        this.msStuckDetectInit = 10000;
    }

    private String LOG_TAG = "Comp Bot TeleOp";

    CompetitionBotConfig robot;
    JoystickButtonUpdated robotDirectionReveseButton;
    boolean robotForward = defaultRobotForwards;

    // Beacon Pushers:

    // Collector Motors:
    private double collectorMotorsSpeed = -1;
    private double collectorMotorsReleaseSpeed = 1;
    private List<DcMotor> collectorMotors;

    // Drive Motors:
    private double driveMotorGain = -0.7;

    // Launcher Motors:
    private JoystickButtonUpdated launcherActivateButton;

    // Feeder Servo:
    private double feederServoManualGain = 1;
    private double feederServoFeedPower = 0.8;
    private boolean feederNotDisabled = false;
    private JoystickButtonUpdated feederServoActivateButton;

    // Feed Detector:
    private double feedDetectorBallValue = 0.03;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBotConfig(hardwareMap, telemetry, null);
        robot.setUpIMU();

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
        feederServoActivateButton = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad2.right_bumper;
            }
        });

        // Beacon Pushers:

        // Collector Motors:
        collectorMotors = new ArrayList<>();
        collectorMotors.add(hardwareMap.dcMotor.get("collector"));

        LinearOpModeUtil.runWhileWait(this, new Callable<Void>() {
            @Override
            public Void call() throws Exception {
                telemetry.addData("Initialization", "Done");
                telemetry.update();
                return null;
            }
        });

        while (opModeIsActive()) {
            // Collector Motors:
            if (gamepad1.right_bumper) {
                DcMotorUtil.setMotorsPower(collectorMotors, collectorMotorsSpeed);
            } else if (gamepad1.left_bumper) {
                DcMotorUtil.setMotorsPower(collectorMotors, collectorMotorsReleaseSpeed);
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
            double feederSpeed = gamepad2.right_trigger * feederServoManualGain;

            //  Read the feed detector:
            double feedDetectorValue = robot.feedDetector.getLightDetected();
            if (feederSpeed == 0) {
                //  Get current button data:
                JoystickButtonUpdated.JoystickButtonData joystickButtonData
                        = feederServoActivateButton.getValueIgnoreException();

                //  If the button press is new, enable the feeder:
                if (joystickButtonData.buttonState && joystickButtonData.isButtonStateNew) {
                    feederNotDisabled = true;
                }
                //  Disable the feeder, if the feed detector is above the desired value:
                if (feedDetectorValue > feedDetectorBallValue) {
                    feederNotDisabled = false;
                }

                //  Default feeder speed:
                feederSpeed = 0;
                //  Turn on the feeder if the button is being pushed, and the feeder is enabled:
                if (joystickButtonData.buttonState && feederNotDisabled) {
                    feederSpeed = feederServoFeedPower;
                }
                //  Set the feeder speed:
            }
            robot.feederServo.setPower(feederSpeed);

            //  Add feeder speed to telemetry:
            telemetry.addData("Feeder Servo Speed", feederSpeed);

            //  Add feeder detector value to log:
            telemetry.addData("Feed Detector", feedDetectorValue);

            telemetry.update();
        }
    }
}
