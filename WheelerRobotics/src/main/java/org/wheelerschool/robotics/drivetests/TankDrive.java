package org.wheelerschool.robotics.drivetests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by luciengaitskell on 10/23/17.
 */

public class TankDrive extends OpMode {
    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontRight;

    @Override
    public void init() {
        telemetry.addData("Status", "Start Init");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        telemetry.addData("Status", "Finish Init");
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Loop");
 
        float leftPower = gamepad1.left_stick_y;
        // Left:
        motorBackLeft.setPower(leftPower);
        motorFrontLeft.setPower(leftPower);

        float rightPower = gamepad1.right_stick_y;
        // Right:
        motorBackRight.setPower(rightPower);
        motorFrontRight.setPower(rightPower);

        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Joy a", gamepad1.a);
    }
}
