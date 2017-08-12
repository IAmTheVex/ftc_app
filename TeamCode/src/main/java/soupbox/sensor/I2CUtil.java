package soupbox.sensor;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.I2cWaitControl;

import java.nio.ByteBuffer;

public class I2CUtil {
    private final I2cDeviceSynch devSynch;

    public I2CUtil(I2cDevice d) {
        devSynch = new I2cDeviceSynchImpl(d, false);
    }

    public void writeReg(int reg, int byteVal) {
        devSynch.write8(reg, byteVal, I2cWaitControl.WRITTEN);
    }

    public int readReg(int reg) {
        return devSynch.read8(reg);
    }

    public void writeReg32Bit(int register, int value) {
        final byte[] data = ByteBuffer.allocate(4).putInt(value).array();
        devSynch.write(register, data, I2cWaitControl.WRITTEN);
    }

    public void writeReg16Bit(int register, int value) {
        final byte[] data = ByteBuffer.allocate(2).putInt(value).array();
        devSynch.write(register, data, I2cWaitControl.WRITTEN);
    }

    public int readReg16Bit(int register) {
        final byte[] buf = devSynch.read(register, 2);
        return ByteBuffer.wrap(buf).getInt();
    }


    public void readMulti(int register, int[] bytes) {
        final byte[] rb = devSynch.read(register, bytes.length);
        for (int i = 0; i < bytes.length; ++i) {
            bytes[i] = 0xFF & rb[i];
        }
    }

    public void writeMulti(int register, int bytes[]) {
        final byte[] wb = new byte[bytes.length];
        for (int i = 0; i < bytes.length; ++i) {
            wb[i] = (byte)(bytes[i] & 0xFF);
        }
        devSynch.write(register, wb, I2cWaitControl.WRITTEN);
    }
}
