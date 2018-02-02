package org.wheelerschool.robotics.competitionbot.library;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.wheelerschool.robotics.library.motion.MechanumDrive4x;
import org.wheelerschool.robotics.library.motion.PositionalMotor;
import org.wheelerschool.robotics.library.motion.ServoTwoPos;

/**
 * Created by luciengaitskell on 10/28/17.
 */

public class CompetitionBot {
    private HardwareMap hw;

    // Drive Motors:
    public MechanumDrive4x driveMotors;

    // Glypht:
    public PositionalMotor glyphtDrive;

    public ServoTwoPos glyphGrabber;

    public final double glyphIntakePower = 0.8;
    private ServoTwoPos rightGlyphRaise;
    private CRServo rightGlyphIntake;
    private ServoTwoPos leftGlyphRaise;
    private CRServo leftGlyphIntake;

    private boolean glyphState = false;

    // Relic:
    public PositionalMotor relicExtension;
    public ServoTwoPos relicGrabber;
    public ServoTwoPos relicWrist;

    // Jewel:
    public Servo jewelWrist;
    public static class JewelWristPositions {
        public static float UP = 0.14f;
        public static float CLEAR = 0.17f;
        public static float EXT = 0.7f;
    }
    public ColorSensor jewelColor;

    private DcMotor setupDcMotor(DcMotor m, DcMotorSimple.Direction d) {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setDirection(d);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return m;
    }

    private static void resetEncoder(DcMotor m) {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetDriveMotors() {
        resetEncoder(driveMotors.bLeft);
        resetEncoder(driveMotors.bRight);
        resetEncoder(driveMotors.fLeft);
        resetEncoder(driveMotors.fRight);
    }

    private void setupDevices() {
        // Drive Motors:
        driveMotors = new MechanumDrive4x(
                setupDcMotor(hw.dcMotor.get("motorFrontLeft"), DcMotorSimple.Direction.REVERSE),
                setupDcMotor(hw.dcMotor.get("motorFrontRight"), DcMotorSimple.Direction.FORWARD),
                setupDcMotor(hw.dcMotor.get("motorBackLeft"), DcMotorSimple.Direction.REVERSE),
                setupDcMotor(hw.dcMotor.get("motorBackRight"), DcMotorSimple.Direction.FORWARD));

        // Glypht:
        glyphtDrive = new PositionalMotor(hw.dcMotor.get("glyphtDrive"), new int[]{-11000, -5100, 0}, 1);
        glyphtDrive.dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        glyphtDrive.dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //glyphGrabber = new ServoTwoPos(hw.servo.get("glyphGrabber"), 0, 0.5);
        glyphGrabber = new ServoTwoPos(hw.servo.get("glyphGrabber"), 0, 0.9);

        rightGlyphRaise = new ServoTwoPos(hw.servo.get("rightGlyphRaise"), 0.15, 0.6);
        rightGlyphRaise.s.setDirection(Servo.Direction.REVERSE);
        rightGlyphIntake = hw.crservo.get("rightGlyphIntake");

        leftGlyphRaise = new ServoTwoPos(hw.servo.get("leftGlyphRaise"), 0.05, 0.5);
        leftGlyphIntake = hw.crservo.get("leftGlyphIntake");
        leftGlyphIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Relic:
        relicExtension = new PositionalMotor(hw.dcMotor.get("relicExtension"), new int[]{0, 3000, 9200});
        relicExtension.dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        relicExtension.dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicGrabber = new ServoTwoPos(hw.servo.get("relicGrabber"), 0.5, 0.05);
        relicGrabber.setState(true);
        relicWrist = new ServoTwoPos(hw.servo.get("relicWrist"), 0, 1);
        relicWrist.setState(true);

        // Jewel:
        jewelWrist = hw.servo.get("jewelWrist");
        jewelWrist.setPosition(JewelWristPositions.UP);
        jewelColor = hw.colorSensor.get("jewelColor");

        setGlyphRaiseState(glyphState);
    }

    public void resetDevices() {
        // Glypht:
        glyphtDrive.dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphtDrive.dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Relic:
        relicExtension.dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicExtension.dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public CompetitionBot(HardwareMap hw) {
        this.hw = hw;
        setupDevices();
    }

    public void setGlyphRaiseState(boolean s) {
        if (!s) {
            setGlyphIntake(0);
        }
        glyphState = s;
        rightGlyphRaise.setState(s);
        leftGlyphRaise.setState(s);
    }

    public void setGlyphIntake(double power) {
        double resultPower = 0;
        if (glyphState) {
            resultPower = power;
        }

        rightGlyphIntake.setPower(resultPower);
        leftGlyphIntake.setPower(resultPower);
    }
}
