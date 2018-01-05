package org.wheelerschool.robotics.competitionbot.library;

import com.qualcomm.robotcore.hardware.CRServo;
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

    private final double glyphIntakePower = 0.5;
    private ServoTwoPos rightGlyphRaise;
    private CRServo rightGlyphIntake;
    private ServoTwoPos leftGlyphRaise;
    private CRServo leftGlyphIntake;

    private boolean glyphState = false;

    // Relic:
    private DcMotor relicExtension;

    private DcMotor setupDcMotor(DcMotor m, DcMotorSimple.Direction d) {
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setDirection(d);
        return m;
    }

    private void setupDevices() {
        // Drive Motors:
        driveMotors = new MechanumDrive4x(
                setupDcMotor(hw.dcMotor.get("motorFrontLeft"), DcMotorSimple.Direction.REVERSE),
                setupDcMotor(hw.dcMotor.get("motorFrontRight"), DcMotorSimple.Direction.FORWARD),
                setupDcMotor(hw.dcMotor.get("motorBackLeft"), DcMotorSimple.Direction.REVERSE),
                setupDcMotor(hw.dcMotor.get("motorBackRight"), DcMotorSimple.Direction.FORWARD));

        // Glypht:
        glyphtDrive = new PositionalMotor(hw.dcMotor.get("glyphtDrive"), new int[]{-9650, 0}, 1);
        glyphtDrive.dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphtDrive.dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        glyphtDrive.dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightGlyphRaise = new ServoTwoPos(hw.servo.get("rightGlyphRaise"), 0.2, 0.6);
        rightGlyphRaise.s.setDirection(Servo.Direction.REVERSE);
        rightGlyphIntake = hw.crservo.get("rightGlyphIntake");

        leftGlyphRaise = new ServoTwoPos(hw.servo.get("leftGlyphRaise"), 0.1, 0.5);
        leftGlyphIntake = hw.crservo.get("leftGlyphIntake");
        leftGlyphIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        setGlyphRaiseState(glyphState);

        // Relic:
        relicExtension = hw.dcMotor.get("relicExtension");
    }

    public CompetitionBot(HardwareMap hw) {
        this.hw = hw;
        setupDevices();
    }

    public void setGlyphRaiseState(boolean s) {
        if (!s) {
            setGlyphIntakeState(false);
        }
        glyphState = s;
        rightGlyphRaise.setState(s);
        leftGlyphRaise.setState(s);
    }

    public void setGlyphIntakeState(boolean s) {
        if (glyphState) {
            double p = 0;
            if (s) {
                p = glyphIntakePower;
            }

            rightGlyphIntake.setPower(p);
            leftGlyphIntake.setPower(p);
        }
    }
}
