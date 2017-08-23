package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDeviceImpl;

/**
 * Created by FTC on 6/6/2017.
 */
public class IMU extends I2cControllerPortDeviceImpl implements HardwareDevice {

    I2cAddr address = I2cAddr.create8bit(0x28);

    public static final int CONFIGMODE = 0b0000;
    public static final int ACCONLY = 0b0001;
    public static final int MAGONLY = 0b0010;
    public static final int GYROONLY = 0b0011;
    public static final int ACCMAG = 0b0100;
    public static final int ACCGYRO = 0b0101;
    public static final int MAGGYRO = 0b0110;
    public static final int AMG = 0b0111;
    public static final int IMU = 0b1000;
    public static final int COMPASS = 0b1001;
    public static final int M4G = 0b1010;
    public static final int NDOF_FMC_OFF = 0b1011;
    public static final int NDOF = 0b1100;

    public static final int CHIP_ID = 0x0;
    public static final int ACC_ID = 0x1;
    public static final int MAG_ID = 0x2;
    public static final int GYR_ID = 0x3;
    public static final int SW_REV_ID_LSB = 0x4;
    public static final int SW_REV_ID_MSB = 0x5;
    public static final int BL_REV_ID = 0x6;
    public static final int PAGE_ID = 0x7;
    public static final int ACC_DATA_X_LSB = 0x8;
    public static final int ACC_DATA_X_MSB = 0x9;
    public static final int ACC_DATA_Y_LSB = 0xa;
    public static final int ACC_DATA_Y_MSB = 0xb;
    public static final int ACC_DATA_Z_LSB = 0xc;
    public static final int MAG_DATA_X_LSB = 0xe;
    public static final int MAG_DATA_X_MSB = 0xf;
    public static final int MAG_DATA_Y_LSB = 0x10;
    public static final int MAG_DATA_Y_MSB = 0x11;
    public static final int MAG_DATA_Z_LSB = 0x12;
    public static final int MAG_DATA_Z_MSB = 0x13;
    public static final int GYR_DATA_X_LSB = 0x14;
    public static final int GYR_DATA_X_MSB = 0x15;
    public static final int GYR_DATA_Y_LSB = 0x16;
    public static final int GYR_DATA_Y_MSB = 0x17;
    public static final int GYR_DATA_Z_LSB = 0x18;
    public static final int GYR_DATA_Z_MSB = 0x19;
    public static final int EUL_HEADING_LSB = 0x1a;
    public static final int EUL_HEADING_MSB = 0x1b;
    public static final int EUL_ROLL_LSB = 0x1c;
    public static final int EUL_ROLL_MSB = 0x1d;
    public static final int EUL_PITCH_LSB = 0x1e;
    public static final int EUL_PITCH_MSB = 0x1f;
    public static final int QUA_DATA_W_LSB = 0x20;
    public static final int QUA_DATA_W_MSB = 0x21;
    public static final int QUA_DATA_X_LSB = 0x22;
    public static final int QUA_DATA_X_MSB = 0x23;
    public static final int QUA_DATA_Y_LSB = 0x24;
    public static final int QUA_DATA_Y_MSB = 0x25;
    public static final int QUA_DATA_Z_LSB = 0x26;
    public static final int QUA_DATA_Z_MSB = 0x27;
    public static final int LIA_DATA_X_LSB = 0x28;
    public static final int LIA_DATA_X_MSB = 0x29;
    public static final int LIA_DATA_Y_LSB = 0x2a;
    public static final int LIA_DATA_Y_MSB = 0x2b;
    public static final int LIA_DATA_Z_LSB = 0x2c;
    public static final int LIA_DATA_Z_MSB = 0x2d;
    public static final int GRV_DATA_X_LSB = 0x2e;
    public static final int GRV_DATA_X_MSB = 0x2f;
    public static final int GRV_DATA_Y_LSB = 0x30;
    public static final int GRV_DATA_Y_MSB = 0x31;
    public static final int GRV_DATA_Z_LSB = 0x32;
    public static final int GRV_DATA_Z_MSB = 0x33;
    public static final int TEMP = 0x34;
    public static final int CALIB_STAT = 0x35;
    public static final int ST_RESULT = 0x36;
    public static final int INT_STA = 0x37;
    public static final int SYS_CLK_STATUS = 0x38;
    public static final int SYS_STATUS = 0x39;
    public static final int SYS_ERR = 0x3a;
    public static final int UNIT_SEL = 0x3b;
    public static final int OPR_MODE = 0x3d;
    public static final int PWR_MODE = 0x3e;
    public static final int SYS_TRIGGER = 0x3f;
    public static final int TEMP_SOURCE = 0x40;
    public static final int AXIS_MAP_CONFIG = 0x41;
    public static final int AXIS_MAP_SIGN = 0x42;



    protected IMU(I2cController controller, int physicalPort) {
        super(controller, physicalPort);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "BNO055 IMU";
    }

    @Override
    public String getConnectionInfo() {
        return "Bosch BNO055 9-axis IMU at 0x28";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
