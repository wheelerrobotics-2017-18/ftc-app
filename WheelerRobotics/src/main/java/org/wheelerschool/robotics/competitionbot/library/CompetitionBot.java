package org.wheelerschool.robotics.competitionbot.library;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.wheelerschool.robotics.library.util.DcMotorGroup;

/**
 * Created by luciengaitskell on 10/28/17.
 */

public class CompetitionBot {
    private HardwareMap hw;

    // Drive Motors:
    private DcMotorGroup leftMotors = new DcMotorGroup();
    private DcMotorGroup rightMotors = new DcMotorGroup();

    private void setupDevices() {
        // Drive Motors:
        leftMotors.add(hw.dcMotor.get("leftFrontMotor"));
        leftMotors.add(hw.dcMotor.get("leftBackMotor"));
        rightMotors.add(hw.dcMotor.get("rightFrontMotor"));
        rightMotors.add(hw.dcMotor.get("rightBackMotor"));
    }

    public CompetitionBot(HardwareMap hw) {
        this.hw = hw;
        setupDevices();
    }

    public void setDrivePower(float lPower, float rPower) {
        leftMotors.setMotorsPower(lPower);
        rightMotors.setMotorsPower(rPower);
    }
}
