package org.wheelerschool.robotics.library.ultrasoundSensor;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.wheelerschool.robotics.library.ollie.Wire;

/**
 * Created by lucien on 12/30/15.
 *
 * @author Lucien Gaitskell
 * @version 0.1
 */
public class MaxSonarSensor implements Runnable {
    private Wire ds;
    private int readCount=0;
    private int distance;
    private int returnDistance=0;
    private int previousDistance=0;
    private long pingTime;
    private boolean running=true;
    public boolean log=false;

    public MaxSonarSensor(HardwareMap hardwareMap, String name, int address){
        ds = new Wire(hardwareMap, name, address);

        new Thread(this).start();
    }

    public MaxSonarSensor(HardwareMap hardwareMap, String name){
        this(hardwareMap, name, 0xE0);
    }

    public void run() {
        ds.beginWrite(0x51);
        ds.write(0);
        ds.endWrite();
        pingTime = System.currentTimeMillis();

        while (running) {
            if ((System.currentTimeMillis() - pingTime) > 100) {
                ds.requestFrom(0, 2);
                ds.beginWrite(0x51);
                ds.write(0);
                ds.endWrite();
                pingTime = System.currentTimeMillis();
            }

            if (ds.responseCount() > 0) {
                ds.getResponse();
                if (ds.isRead()
                        ) {
                    long micros = ds.micros();
                    distance = ds.readHL();
                    if (distance < 760) {
                        readCount++;
                        returnDistance=distance;

                        if (log){
                            Log.d("distance", String.valueOf(distance));

                            int change = Math.abs(previousDistance-returnDistance);
                            if (change>20) {
                                Log.d("distanceWarn", "Thats a big change! (" + String.valueOf(change) + ")");
                            }
                        }
                        previousDistance=returnDistance;
                    }
                }
            }
        }
    }

    public int getDistance(){
        return returnDistance;
    }

    public void close(){
        running=false;
        ds.close();
    }

    @Override
    protected void finalize() throws Throwable {
        close();
    }
}
