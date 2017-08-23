package org.firstinspires.ftc.teamcode.framework.usb;

/**
 * Created by FTC on 8/15/2017.
 */
public class CrcUSB {

    int crc5Polynomial = 0b00101;
    int crc16Polynomial = 0x8005;


    public CrcUSB() {
        // Nothing, really.
    }

    public int crc5(int data) {
        int register = 0b11111;
        int highestBit = Integer.highestOneBit(data);
        int hBPos;
        for (hBPos = 0; (highestBit >> hBPos) == 1; hBPos++) { }

        for (int i = 0; i < hBPos; i++) {
            int bit = BitUtils.bitAt(data, i);

            if (bit == BitUtils.bitAt(register, 4)) {
                register = BitUtils.push(register, 0);
            } else {
                register = BitUtils.push(register, 0);
                register = register ^ crc5Polynomial;
            }
        }

        register = (~register & 0b11111);
        return register;
    }

    public long crc16(long data) {
        long register = 0xffff;
        long highestBit = Long.highestOneBit(data);
        long hBPos;
        for (hBPos = 0; (highestBit >> hBPos) == 1; hBPos++) { }

        for (int i = 0; i < hBPos; i++) {
            long bit = BitUtils.bitAt(data, i);

            if (bit == BitUtils.bitAt(register, 4)) {
                register = BitUtils.push(register, 0);
            } else {
                register = BitUtils.push(register, 0);
                register = register ^ crc16Polynomial;
            }
        }

        register = (~register & 0xffff);
        return register;
    }
}
