package org.firstinspires.ftc.teamcode.framework.usb;

/**
 * Created by FTC on 8/15/2017.
 */
public class BitUtils {

    public static int bitAt(int binary, int pos) {
        if (pos < 0) throw new IllegalArgumentException("Negative pos");
        return (binary >> pos) & 0b1;
    }

    public static long bitAt(long binary, long pos) {
        if (pos < 0) throw new IllegalArgumentException("Negative pos");
        return (binary >> pos) & 0b1;
    }

    public static int push(int binary, int bit) {
        if (bit < 0 || bit > 1) throw new IllegalArgumentException("Invalid bit");
        return (binary << 1) | bit;
    }

    public static long push(long binary, long bit) {
        if (bit < 0 || bit > 1) throw new IllegalArgumentException("Invalid bit");
        return (binary << 1) | bit;
    }
}
