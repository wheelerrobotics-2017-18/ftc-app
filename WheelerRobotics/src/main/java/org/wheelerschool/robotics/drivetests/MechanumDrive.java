package org.wheelerschool.robotics.drivetests;

/**
 * Created by luciengaitskell on 11/20/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.wheelerschool.robotics.library.motion.MechanumDrive4x;

@TeleOp
public class MechanumDrive extends OpMode {

    private MechanumDrive4x md;

    DcMotor motorBackLeft;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontRight;


    @Override
    public void init() {
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        md = new MechanumDrive4x(
                motorFrontLeft,
                motorFrontRight,
                motorBackLeft,
                motorBackRight
        );
    }

    @Override
    public void loop() {
        double[] motorP = md.updateMotors(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x/2);
        telemetry.addData("fLeft", motorP[0]);
        telemetry.addData("fLeft enc", motorFrontLeft.getCurrentPosition());
        telemetry.addData("fRight", motorP[1]);
        telemetry.addData("fRight enc", motorFrontRight.getCurrentPosition());
        telemetry.addData("bLeft", motorP[2]);
        telemetry.addData("bLeft enc", motorBackLeft.getCurrentPosition());
        telemetry.addData("bRight", motorP[3]);
        telemetry.addData("bRight enc", motorBackRight.getCurrentPosition());
    }
}
