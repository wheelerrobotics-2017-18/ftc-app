package org.wheelerschool.robotics.competitionbot.util;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.wheelerschool.robotics.competitionbot.CompetitionBotConfig;

/**
 * Created by luciengaitskell on 1/14/17.
 */

@TeleOp
public class AllSensorRead extends OpMode {
    public AllSensorRead() {
        super();
        this.msStuckDetectInit = 10000;
    }

    CompetitionBotConfig robot;
    UltrasonicSensor leftUltrasound;
    UltrasonicSensor rightUltrasound;
    BNO055IMU imu;

    @Override
    public void init() {
        this.robot = new CompetitionBotConfig(hardwareMap, telemetry, null);
        this.robot.setUpIMU();
        rightUltrasound = hardwareMap.ultrasonicSensor.get("rightUltrasound");
        leftUltrasound = hardwareMap.ultrasonicSensor.get("leftUltrasound");
    }

    @Override
    public void loop() {
        telemetry.addData("Left Ultrasound", leftUltrasound.getUltrasonicLevel());
        telemetry.addData("Right Ultrasound", rightUltrasound.getUltrasonicLevel());
        telemetry.addData("IMU", robot.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle);
        telemetry.addData("Color Left", robot.colorLeft.red() + ", " + robot.colorLeft.green() + ", " + robot.colorLeft.blue());
        telemetry.addData("Color Right", robot.colorRight.red() + ", " + robot.colorRight.green() + ", " + robot.colorRight.blue());
        telemetry.addData("Ground Sensor", robot.groundReflectSensor.getRawLightDetected());
    }
}
