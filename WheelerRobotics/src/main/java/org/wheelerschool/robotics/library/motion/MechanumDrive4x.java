package org.wheelerschool.robotics.library.motion;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by luciengaitskell on 10/19/17.
 */

public class MechanumDrive4x {
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    public MechanumDrive4x(DcMotor fLeft, DcMotor fRight, DcMotor bLeft, DcMotor bRight) {
        this.fLeft = fLeft;
        this.fRight = fRight;
        this.bLeft = bLeft;
        this.bRight = bRight;
    }

    public double[] updateMotors(float x, float y, float rot) {
        double scale = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        final double v1 = scale * Math.cos(robotAngle) + rot;
        final double v2 = scale * Math.sin(robotAngle) - rot;
        final double v3 = scale * Math.sin(robotAngle) + rot;
        final double v4 = scale * Math.cos(robotAngle) - rot;

        /*luc is gei*/
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);

        return new double[] {v1, v2, v3, v4};
    }
}
