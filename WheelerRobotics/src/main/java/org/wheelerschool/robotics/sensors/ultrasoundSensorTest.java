package org.wheelerschool.robotics.sensors;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wheelerschool.robotics.library.ultrasoundSensor.MaxSonarSensor;

/**
 * Created by luciengaitskell on 11/30/16.
 */

@TeleOp
public class ultrasoundSensorTest extends OpMode {
    MaxSonarSensor us;
    @Override
    public void init() {
        us = new MaxSonarSensor(hardwareMap, "US1");
    }

    @Override
    public void loop() {
        telemetry.addData("US dist", us.getDistance());
        Log.d("US dist", Integer.toString(us.getDistance()));
    }

    public void stop(){
        us.close();
    }
}
