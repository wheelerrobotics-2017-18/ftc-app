package org.wheelerschool.robotics.sensors;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.wheelerschool.robotics.library.sensors.MB1242;

import java.util.Arrays;

/**
 * Created by luciengaitskell on 11/30/16.
 */

@TeleOp
public class MB1242Test extends LinearOpMode {
    MB1242 ultrasoundSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        ultrasoundSensor = new MB1242(I2cAddr.create8bit(0xE0), hardwareMap.i2cDevice.get("US1"));

        waitForStart();

        while (opModeIsActive()) {
            ultrasoundSensor.takeReading();
            Thread.sleep(1000);
            Log.d("US Dist", Arrays.toString(ultrasoundSensor.getLastDistance()));
            telemetry.addData("US Dist", Arrays.toString(ultrasoundSensor.getLastDistance()));
            telemetry.update();
        }
    }
}
