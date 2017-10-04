package org.wheelerschool.robotics.library.ultrasoundSensor;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.wheelerschool.robotics.library.ollie.Wire;

/**
 * Created by lucien on 12/30/15.
 *
 * @author Lucien Gaitskell
 * @version 0.1
 */
public class MaxSonarSensorNew implements Runnable {
    private I2cDeviceSynch device;
    public int readCount=0;
    private int distance;
    private int returnDistance=0;
    private int previousDistance=0;
    private long pingTime;
    private boolean running=true;
    public boolean log=false;

    public MaxSonarSensorNew(HardwareMap hardwareMap, String name, int address){
        device = new I2cDeviceSynchImpl(hardwareMap.i2cDevice.get(name), I2cAddr.create8bit(address), false);

        new Thread(this).start();
    }

    public MaxSonarSensorNew(HardwareMap hardwareMap, String name){
        this(hardwareMap, name, 0xE0);
    }

    public void run() {
        device.write8(0x51, 0);

        pingTime = System.currentTimeMillis();

        while (running) {
            if ((System.currentTimeMillis() - pingTime) > 1000) {
                readCount++;
                byte[] data  = device.read(0, 2);

                returnDistance = 256 * data[1] + data[0];
                previousDistance = returnDistance;

                device.write8(0x51, 0);

                pingTime = System.currentTimeMillis();
            }
        }
    }

    public int getDistance(){
        return returnDistance;
    }

    public void close(){
        running=false;
    }

    @Override
    protected void finalize() throws Throwable {
        close();
    }
}
