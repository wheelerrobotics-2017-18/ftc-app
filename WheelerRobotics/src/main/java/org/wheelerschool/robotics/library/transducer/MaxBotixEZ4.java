package org.wheelerschool.robotics.library.transducer;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

/**
 * Created by luciengaitskell on 11/22/17.
 */

public class MaxBotixEZ4 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private final int readDelay = 80;
    private Long lastRead = null;

    public MaxBotixEZ4(I2cDeviceSynch device) {
        this(device, I2cAddr.create7bit(112));
    }

    public MaxBotixEZ4(I2cDeviceSynch device, I2cAddr addr) {
        super(device, true);
        deviceClient.setI2cAddress(addr);
        deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MaxBotix EZ4";
    }

    public boolean takeReading() {
        deviceClient.write(81, new byte[]{0});
        lastRead = System.currentTimeMillis();
        return true;
    }

    public boolean rangeReady() {
        return System.currentTimeMillis() - lastRead >= readDelay;
    }

    public int readRange() {
        if (lastRead == null)
            return -1;
        while (!rangeReady()) {}

        byte[] data = deviceClient.read(0, 2);
        return ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
    }
}
