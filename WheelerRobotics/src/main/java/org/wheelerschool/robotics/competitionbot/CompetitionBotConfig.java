package org.wheelerschool.robotics.competitionbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    private HardwareMap hardwareMap;

    public class AdvancedServo {
        public Servo servo;
        private double retractedPos;
        private double extendedPos;

        public AdvancedServo(Servo servo) {
            this.servo = servo;
        }

        public void activatePusher(boolean extend) {
            if (extend) {
                servo.setPosition(extendedPos);
            } else {
                servo.setPosition(retractedPos);
            }
        }
    }
    // Autonomous Left/Right:
    public AdvancedServo pusherLeft;
    public AdvancedServo pusherRight;

    public static boolean defaultRobotForwards = true;
    private boolean robotForwards = defaultRobotForwards;
    public List<DcMotor> leftMotors;
    public List<DcMotor> rightMotors;

    public CompetitionBotConfig(HardwareMap hardwareMap) {
        this(hardwareMap, defaultRobotForwards);
    }

    public CompetitionBotConfig(HardwareMap hardwareMap, boolean robotForwards) {
        this.hardwareMap = hardwareMap;

        // Pusher Servos:
        this.pusherLeft = new AdvancedServo(hardwareMap.servo.get("pusherLeft"));
        this.pusherLeft.retractedPos = 0;
        this.pusherLeft.extendedPos = 1;
        this.pusherRight = new AdvancedServo(hardwareMap.servo.get("pusherRight"));
        this.pusherRight.servo.setDirection(Servo.Direction.REVERSE);
        this.pusherRight.retractedPos = 0;
        this.pusherRight.extendedPos = 1;

        // Drive Motors:
        this.leftMotors = new ArrayList<>();
        this.leftMotors.add(this.hardwareMap.dcMotor.get("backLeft"));
        this.rightMotors = new ArrayList<>();
        this.rightMotors.add(this.hardwareMap.dcMotor.get("backRight"));
        // Set robot direction to default:
        setRobotDirection(robotForwards);

        DcMotorUtil.setMotorsRunMode(this.leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorUtil.setMotorsRunMode(this.rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRobotDirection(boolean forwards) {
        if (robotForwards != forwards) {
            List<DcMotor> newRightMotors = this.leftMotors;
            List<DcMotor> newLeftMotors = this.rightMotors;
            this.leftMotors = newLeftMotors;
            this.rightMotors = newRightMotors;
        }

        DcMotorUtil.setMotorsDirection(this.leftMotors, DcMotorSimple.Direction.REVERSE);
        DcMotorUtil.setMotorsDirection(this.rightMotors, DcMotorSimple.Direction.FORWARD);
    }
}
