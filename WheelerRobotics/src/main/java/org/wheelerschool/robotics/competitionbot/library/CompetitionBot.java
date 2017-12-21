package org.wheelerschool.robotics.competitionbot.library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.wheelerschool.robotics.library.motion.MechanumDrive4x;

/**
 * Created by luciengaitskell on 10/28/17.
 */

public class CompetitionBot {
    private HardwareMap hw;

    // Drive Motors:
    public MechanumDrive4x driveMotors;

    // Glypht:
    private DcMotor glyphtDrive;

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
        glyphtDrive = hw.dcMotor.get("glyphtDrive");

        // Relic:
        relicExtension = hw.dcMotor.get("relicExtension");
    }

    public CompetitionBot(HardwareMap hw) {
        this.hw = hw;
        setupDevices();
    }
}
