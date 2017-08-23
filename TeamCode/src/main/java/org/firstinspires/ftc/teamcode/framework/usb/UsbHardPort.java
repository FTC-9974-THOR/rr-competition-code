package org.firstinspires.ftc.teamcode.framework.usb;

import android.support.annotation.Nullable;

import com.google.common.primitives.Bytes;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.scurrilous.circe.HashProvider;
import com.scurrilous.circe.crc.StandardCrcProvider;
import com.scurrilous.circe.params.CrcParameters;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by FTC on 8/13/2017.
 *
 * Day 1: Read, write, and connection listener added
 *
 * Day 2: Write bit stuffing implemented, made read/write synchronized methods,
 * implemented (but not yet tested) CRC algorithm, overhauled read/write system
 *
 * TODO:
 * SOF watchdog
 * Hardware
 * Object-oriented abstraction layers
 */

public class UsbHardPort {

    DigitalChannel dPlus;
    DigitalChannel dMinus;

    int J = 0;
    int K = 0;
    int SE0 = 0b00;
    int SE1 = 0b11;

    public static final int NONE = -1;
    public static final int CTRL_READ = 0;
    public static final int CTRL_WRITE = 1;

    CrcUSB crcEngine;

    volatile int state = SE0;

    final int ACK_LENGTH = 19;

    ByteBuffer readBuffer;

    /*public enum State {
        J((byte) 0b01),
        K((byte) 0b10),
        SE0((byte) 0b00),
        SE1((byte) 0b11);

        private byte val;

        State(byte v) {
            this.val = v;
        }

        public byte asByte() {
            return this.val;
        }

        public byte plus() {
            return (byte) (this.val & 0b10);
        }

        public byte minus() {
            return (byte) (this.val & 0b01);
        }
    }*/

    Thread connectDetect;
    Runnable connectRunnable = new Runnable() {
        @Override
        public void run() {
            while (!(dPlus.getState() || dMinus.getState())) {
                try {
                    Thread.sleep(0, 500000);
                } catch (InterruptedException e) {
                    RobotLog.logStacktrace(e);
                }
            }
            J = (((byte) ((dPlus.getState()) ? 1 : 0)) << 1) | (((byte) ((dMinus.getState()) ? 1 : 0)));
            K = ~J;
            init();
        }
    };

    public UsbHardPort(HardwareMap hw) {
        crcEngine = new CrcUSB();

        dPlus = hw.digitalChannel.get("d+");
        dMinus = hw.digitalChannel.get("d-");

        dPlus.setState(false);
        dPlus.setMode(DigitalChannel.Mode.INPUT);

        dMinus.setState(false);
        dMinus.setMode(DigitalChannel.Mode.INPUT);

        connectDetect = new Thread(connectRunnable);
        connectDetect.start();
    }

    private void init() {
        // Send GET_DESCRIPTOR setup packet
        byte[] setup = {
                (byte) 0x2d, // 00101101
                (byte) 0x00, // 00000000
                (byte) 0x00  // 00000000
        };
        byte[] data0 = {
                (byte) 0xc3, // 11000011
                (byte) 0x80, // 10000000
                (byte) 0x06, // 00000110
                (byte) 0x00, // 00000000
                (byte) 0x01, // 00000001
                (byte) 0x00, // 00000000
                (byte) 0x00, // 00000000
                (byte) 0x12, // 00010010
                (byte) 0x00  // 00000000
        };
        byte[] status = {
                (byte) 0xe1, // 11100001
                (byte) 0x00, // 00000000
                (byte) 0x00  // 00000000
        };

        // Wait 100 ms for device power to stabilise
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            RobotLog.logStacktrace(e);
        }

        write(ByteBuffer.wrap(setup), true);
        write(ByteBuffer.wrap(data0), true);
        byte[] in = read(-1, true).array();

        byte pid = Arrays.copyOfRange(in, 8, in.length)[0];

        byte[] descriptorIn = read(-1, true).array();
        byte[] descriptor = Arrays.copyOfRange(descriptorIn, 8, descriptorIn.length);

        byte[] ackS = {
                (byte) 0xd2  // 11010010
        };

        write(ByteBuffer.wrap(ackS), true);
        write(ByteBuffer.wrap(status), true);

        // Now that we have the device descriptor, we can give the device a new address
        // To do that, we send a SET_ADDRESS request

        byte[] addrData0 = {
                (byte) 0xc3, // 11000011
                (byte) 0x00, // 00000000
                (byte) 0x05, // 00000101
                (byte) 0x01, // 00000001
                (byte) 0x00, // 00000000
                (byte) 0x00, // 00000000
                (byte) 0x00, // 00000000
                (byte) 0x00, // 00000000
                (byte) 0x00  // 00000000
        };

        resetDevice(true);

        write(ByteBuffer.wrap(setup), true);
        write(ByteBuffer.wrap(addrData0), true);
        byte[] ack = read(-1, true).array();

        write(ByteBuffer.wrap(setup), true);

        setup[1] = (byte) 0x01; // 00000001
        byte[] configData0 = {
                (byte) 0xc3, // 11000011
                (byte) 0x80, // 10000000
                (byte) 0x06, // 00000110
                (byte) 0x02, // 00000010
                (byte) 0x00, // 00000000
                (byte) 0x00, // 00000000
                (byte) 0x00, // 00000000
                (byte) 0x09, // 00001001
                (byte) 0x00  // 00000000
        };


    }

    synchronized void write(ByteBuffer data, boolean runOnCallingThread) {
        final ByteBuffer workingData = data;
        Thread writeThread = new Thread(new Runnable() {
            @Override
            public void run() {
                int crcMethod = 0;
                switch (workingData.array()[0]) {
                    case ((byte) 225):
                        // OUT
                    case ((byte) 105):
                        // IN
                    case ((byte) 165):
                        // SOF
                    case ((byte) 45):
                        // SETUP
                        crcMethod = 1;
                        break;
                    case ((byte) 195):
                        crcMethod = 2;
                        break;
                }

                dPlus.setMode(DigitalChannel.Mode.OUTPUT);
                dMinus.setMode(DigitalChannel.Mode.OUTPUT);

                for (int i = 0; i < 3; i++) {
                    dPlus.setState((((K & 0b10) >> 1) == 1));
                    dMinus.setState(((K & 0b01) == 1));

                    try {
                        Thread.sleep(0, 83);
                    } catch (InterruptedException e) {
                        RobotLog.logStacktrace(e);
                    }

                    dPlus.setState((((J & 0b10) >> 1) == 1));
                    dMinus.setState(((J & 0b01) == 1));

                    try {
                        Thread.sleep(0, 83);
                    } catch (InterruptedException e) {
                        RobotLog.logStacktrace(e);
                    }
                }

                dPlus.setState((((K & 0b10) >> 1) == 1));
                dMinus.setState(((K & 0b01) == 1));

                try {
                    Thread.sleep(0, 83);
                } catch (InterruptedException e) {
                    RobotLog.logStacktrace(e);
                }

                dPlus.setState((((K & 0b10) >> 1) == 1));
                dMinus.setState(((K & 0b01) == 1));

                try {
                    Thread.sleep(0, 83);
                } catch (InterruptedException e) {
                    RobotLog.logStacktrace(e);
                }



                byte[] packet = workingData.array();

                int consecutiveOnes = 0;

                // The line state should now be K, so set the state variable to K
                state = K;

                byte[] toCrc = Arrays.copyOfRange(packet, 1, packet.length);
                // crcHash can be initialized to have a size of zero
                // because it is overwritten anyway
                byte[] crcHash = new byte[0];
                if (crcMethod == 1) {
                    int crc5Data = (toCrc[0] << 4) | (toCrc[1] & 0b1111);
                    crcHash = new byte[] {(byte) crcEngine.crc5(crc5Data)};
                } else if (crcMethod == 2) {
                    long crc16Data = 0;
                    for (byte b : toCrc) {
                        crc16Data = (crc16Data << 8) | b;
                    }
                    long crc16Hash = crcEngine.crc16(crc16Data);
                    crcHash = new byte[] {(byte) (crc16Hash >> 8), (byte) (crc16Hash & 0xff)};
                }

                if (crcHash.length == 0) {
                    RobotLog.e("CRC hashing failed");
                }
                packet = Bytes.concat(packet, crcHash);
                packet = ByteBuffer.wrap(packet).order(ByteOrder.LITTLE_ENDIAN).array();


                int j = 0;
                for (byte b : packet) {
                    int shifts = 8;
                    if (crcMethod == 1) {
                        if (j == 0) {
                            shifts = 7;
                        }
                        if (j == 1) {
                            shifts = 4;
                        }
                    }
                    j++;
                    for (int i = 0; i < shifts; i++) {
                        int bit = (b >> i) & 0b1;

                        if (bit == 1) {
                            consecutiveOnes++;
                        } else {
                            state = ~state;
                            consecutiveOnes = 0;
                        }

                        dPlus.setState((((state & 0b10) >> 1) == 1));
                        dMinus.setState(((state & 0b01) == 1));

                        try {
                            Thread.sleep(0, 83);
                        } catch (InterruptedException e) {
                            RobotLog.logStacktrace(e);
                        }

                        // If the just-sent bit was a 6th consecutive 1, bit stuff
                        if (consecutiveOnes == 6) {
                            // Commence the bit stuffing!
                            state = ~state;
                            dPlus.setState((((state & 0b10) >> 1) == 1));
                            dMinus.setState(((state & 0b01) == 1));

                            try {
                                Thread.sleep(0, 83);
                            } catch (InterruptedException e) {
                                RobotLog.logStacktrace(e);
                            }
                            consecutiveOnes = 0;
                        }
                    }
                }

                state = SE0;

                dPlus.setState((((state & 0b10) >> 1) == 1));
                dMinus.setState(((state & 0b01) == 1));

                try {
                    Thread.sleep(0, 166);
                } catch (InterruptedException e) {
                    RobotLog.logStacktrace(e);
                }

                state = J;

                dPlus.setState((((state & 0b10) >> 1) == 1));
                dMinus.setState(((state & 0b01) == 1));

                try {
                    Thread.sleep(0, 83);
                } catch (InterruptedException e) {
                    RobotLog.logStacktrace(e);
                }

                dPlus.setMode(DigitalChannel.Mode.INPUT);
                dMinus.setMode(DigitalChannel.Mode.INPUT);
            }
        });
        if (runOnCallingThread) {
            writeThread.run();
        } else {
            writeThread.start();
        }
    }

    synchronized ByteBuffer read(int bytes, boolean runOnCallingThread) {
        final int _bytes = bytes;
        Thread readThread = new Thread(new Runnable() {
            @Override
            public void run() {
                boolean readToEnd = (_bytes == -1);
                ByteBuffer buffer;
                List<Byte> dynamicBuffer;
                dPlus.setMode(DigitalChannel.Mode.INPUT);
                dMinus.setMode(DigitalChannel.Mode.INPUT);

                if (readToEnd) {
                    dynamicBuffer = new ArrayList<>();
                    int newState = J;
                    while (newState == J) {
                        int dPlusIn = (dPlus.getState()) ? 1 : 0;
                        int dMinusIn = (dMinus.getState()) ? 1 : 0;

                        newState = (dPlusIn << 1) | (dMinusIn);
                    }
                    boolean hasRecievedSE0 = false;
                    while (!hasRecievedSE0 && newState != J) {
                        String bits = "";
                        for (int j = 0; j < 8; j++) {
                            int dPlusIn = (dPlus.getState()) ? 1 : 0;
                            int dMinusIn = (dMinus.getState()) ? 1 : 0;

                            newState = (dPlusIn << 1) | (dMinusIn);

                            if (newState == SE0 && !hasRecievedSE0) {
                                hasRecievedSE0 = true;
                            }

                            if (newState == SE1) {
                                RobotLog.setGlobalErrorMsg("SE1 detected");
                            }

                            if (!hasRecievedSE0) {
                                if (newState != state) {
                                    bits = bits + "1";
                                } else {
                                    bits = bits + "0";
                                }
                            }
                            state = newState;

                            try {
                                Thread.sleep(0, 83);
                            } catch (InterruptedException e) {
                                RobotLog.logStacktrace(e);
                            }
                        }
                        dynamicBuffer.add(Byte.parseByte(bits, 2));
                    }
                    readBuffer = ByteBuffer.wrap(Bytes.toArray(dynamicBuffer)).order(ByteOrder.BIG_ENDIAN);
                } else {
                    buffer = ByteBuffer.allocate(_bytes);
                    buffer.order(ByteOrder.LITTLE_ENDIAN);
                    for (int i = 0; i < buffer.capacity(); i++) {
                        String bits = "";
                        for (int j = 0; j < 8; j++) {
                            int dPlusIn = (dPlus.getState()) ? 1 : 0;
                            int dMinusIn = (dMinus.getState()) ? 1 : 0;

                            int newState = (dPlusIn << 1) | (dMinusIn);

                            if (newState == SE1) {
                                RobotLog.setGlobalErrorMsg("SE1 detected");
                            }

                            if (newState != state) {
                                bits = bits + "1";
                            } else {
                                bits = bits + "0";
                            }
                            state = newState;

                            try {
                                Thread.sleep(0, 83);
                            } catch (InterruptedException e) {
                                RobotLog.logStacktrace(e);
                            }
                        }
                        buffer.put(Byte.parseByte(bits, 2));
                    }
                    buffer.order(ByteOrder.BIG_ENDIAN);
                    readBuffer = buffer;
                }
            }
        });
        if (runOnCallingThread) {
            readThread.run();
        } else {
            readThread.start();
        }
        return readBuffer;
    }

    synchronized void resetDevice(boolean runOnCallingThread) {
        Thread resetThread = new Thread(new Runnable() {
            @Override
            public void run() {
                dPlus.setMode(DigitalChannel.Mode.OUTPUT);
                dMinus.setMode(DigitalChannel.Mode.OUTPUT);

                state = SE0;

                dPlus.setState((((state & 0b10) >> 1) == 1));
                dMinus.setState(((state & 0b01) == 1));

                //long start = System.nanoTime();
                //while (System.nanoTime() - start < 2500000)
                try {
                    Thread.sleep(3);
                } catch (InterruptedException e) {
                    RobotLog.logStacktrace(e);
                }

                dPlus.setMode(DigitalChannel.Mode.INPUT);
                dMinus.setMode(DigitalChannel.Mode.INPUT);

                state = J;
            }
        });
        if (runOnCallingThread) {
            resetThread.run();
        } else {
            resetThread.start();
        }
    }
}
