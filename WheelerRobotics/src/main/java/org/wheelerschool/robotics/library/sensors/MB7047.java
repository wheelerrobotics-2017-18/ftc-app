package org.wheelerschool.robotics.library.sensors;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by luciengaitskell on 11/5/16.
 */

public class MB7047 implements HardwareDevice {
    static I2cAddr DEFAULT_I2C_ADDR = I2cAddr.create7bit(0x70);

    I2cAddr i2cAddr;
    I2cDeviceSynch device;

    public MB7047(I2cDevice i2cDevice){
        this(DEFAULT_I2C_ADDR, i2cDevice);
    }

    public MB7047(I2cAddr i2cAddr, I2cDevice i2cDevice){
        this.i2cAddr = i2cAddr;
        this.device = new I2cDeviceSynchImpl(i2cDevice, i2cAddr, false);
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    // REQUIRED INFORMATION:
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "12CXL-MaxSonar-WR (MB7047)";
    }

    @Override
    public String getConnectionInfo() {
        return "I2c";
    }

    @Override
    public int getVersion() {
        return 1;
    }
}
