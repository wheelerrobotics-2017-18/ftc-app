package org.wheelerschool.robotics.competitionbot.library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.wheelerschool.robotics.library.motion.MechanumDrive4x;
import org.wheelerschool.robotics.library.motion.ServoTwoPos;

/**
 * Created by luciengaitskell on 10/28/17.
 */

public class CompetitionBot {
    private HardwareMap hw;

    // Drive Motors:
    public MechanumDrive4x driveMotors;

    // Glypht:
    private DcMotor glyphtDrive;
    private Servo glyphtGrabberServo;

    // Relic:
    private DcMotor relicExtension;
    private Servo relicFlickServo;
    private Servo relicGrabberServo;

    private DcMotor setupDcMotor(DcMotor m, DcMotorSimple.Direction d) {
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setDirection(d);
        return m;
    }

    private void setupDevices() {
        // Drive Motors:
        driveMotors = new MechanumDrive4x(
                setupDcMotor(hw.dcMotor.get("motorFrontLeft"), DcMotorSimple.Direction.FORWARD),
                setupDcMotor(hw.dcMotor.get("motorFrontRight"), DcMotorSimple.Direction.REVERSE),
                setupDcMotor(hw.dcMotor.get("motorBackLeft"), DcMotorSimple.Direction.FORWARD),
                setupDcMotor(hw.dcMotor.get("motorBackRight"), DcMotorSimple.Direction.REVERSE));

        // Glypht:
        glyphtDrive = hw.dcMotor.get("glyphtDrive");
        glyphtGrabberServo = hw.servo.get("frontGrabberServo");
        gyphtGrabber = new ServoTwoPos(gyphtGrabberServo, 0, 1);

        // Relic:
        relicExtension = hw.dcMotor.get("relicExtension");
        relicFlickServo = hw.servo.get("relicFlickServo");
        relicGrabberServo = hw.servo.get("relicGrabberServo");
    }

    public CompetitionBot(HardwareMap hw) {
        this.hw = hw;
        setupDevices();
    }
}
