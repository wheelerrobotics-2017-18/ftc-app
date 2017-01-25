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
    CompetitionBotConfig robot;
    UltrasonicSensor leftUltrasound;
    UltrasonicSensor rightUltrasound;
    BNO055IMU imu;

    @Override
    public void init() {
        this.robot = new CompetitionBotConfig(hardwareMap, telemetry, null);
        rightUltrasound = hardwareMap.ultrasonicSensor.get("rightUltrasound");
        leftUltrasound = hardwareMap.ultrasonicSensor.get("leftUltrasound");

        //          IMU:
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //          Retrieve and initialize the IMU:
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Ultrasound", leftUltrasound.getUltrasonicLevel());
        telemetry.addData("Right Ultrasound", rightUltrasound.getUltrasonicLevel());
        telemetry.addData("IMU", imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle);
        telemetry.addData("Color Left", robot.colorLeft.red() + ", " + robot.colorLeft.green() + ", " + robot.colorLeft.blue());
        telemetry.addData("Color Right", robot.colorRight.red() + ", " + robot.colorRight.green() + ", " + robot.colorRight.blue());
        telemetry.addData("Ground Sensor", robot.groundReflectSensor.getRawLightDetected());
    }
}
