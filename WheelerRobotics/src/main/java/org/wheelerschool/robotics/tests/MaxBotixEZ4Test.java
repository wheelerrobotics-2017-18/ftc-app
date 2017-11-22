package org.wheelerschool.robotics.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.wheelerschool.robotics.library.transducer.MaxBotixEZ4;

/**
 * Created by luciengaitskell on 11/22/17.
 */


public class MaxBotixEZ4Test extends OpMode {
    MaxBotixEZ4 mb;

    @Override
    public void init() {
        mb = new MaxBotixEZ4(hardwareMap.i2cDeviceSynch.get("us1"));
    }

    @Override
    public void loop() {
        mb.takeReading();
        telemetry.addData("Range", mb.readRange());
    }
}
