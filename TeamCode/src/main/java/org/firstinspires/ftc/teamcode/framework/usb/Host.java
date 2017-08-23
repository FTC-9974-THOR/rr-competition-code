package org.firstinspires.ftc.teamcode.framework.usb;

/**
 * Created by FTC on 6/17/2017.
 */
public class Host {
    /*
        With D- pullup:
        J = D+ low, D- high
        K = D+ high, D- low
        SE0 = D+ low, D- low
        SE1 = D+ high, D- high
        With D+ pullup:
        J = D+ high, D- low
        K = D+ low, D- high
        SE0 = D+ low, D- low
        SE1 = D+ high, D- high
        SE1 is an illegal state!

        USB 1.x LS:
        Disconnected = SE0 for longer than 2 microseconds
        Connection = Device pulls up D-
        Idle = SE0
        Sync = line transition of K-J-K-J-K-J-K-K
        EOP (End Of Packet) = line transition of SE0-SE0-J
        Reset = SE0 longer than 2.5 milliseconds
        Suspend = J for longer than 3msj
        Resume (host) = K >= 20ms and then an EOP
        Resume (device) = device sends K >= 1ms and waits for host resume
        Keep alive = EOP every millisecond

        USB 1.x FS:
        Disconnected = SE0 for longer than 2 microseconds
        Connection = Device pulls up D+
        Idle = SE0
        Sync = line transition of K-J-K-J-K-J-K-K
        EOP (End Of Packet) = line transition of SE0-SE0-J
        Reset = SE0 longer than 2.5 milliseconds
        Suspend = J for longer than 3msj
        Resume (host) = K >= 20ms and then an EOP
        Resume (device) = device sends K >= 1ms and waits for host resume

        USB 2.x:
        Disconnected = SE0 >= 2 microsec.
        Connection = "chirping sequence"
        Idle = ?
        Sync = K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-J-K-K
        EOP = ?
        Reset = ?
        Suspend = ?
        Resume (host) = ?
        Resume (device) = ?

        0 is sent by toggling between J and K (ie. J -> K, K -> J)
        1 is sent by leaving the lines alone

        == Setup and control endpoints ==
        A control transfer is made up of a setup stage,
        an optional data stage, and a status stage.

        Setup packet format:
        Byte 0 -
            Bit 7 = Request direction
            Bits 5-6 = Request type
            Bits 0-4 = Recipient
        Byte 1 - Request
        Byte 2 - Lower byte of request-specific word
        Byte 3 - Upper byte of said word
        Byte 4 - Lower byte of index; generally refers to endpoint index
        Byte 5 - Index upper byte
        Byte 6 - Lower byte of data stage length
        Byte 7 - Upper byte of data stage length

        Requests (request codes):
        GET_STATUS = 0
        CLEAR_FEATURE = 1
        2 is reserved
        SET_FEATURE = 3
        4 is also reserved
        SET_ADDRESS = 5
        GET_DESCRIPTOR = 6
        SET_DESCRIPTOR = 7
        GET_CONFIGURATION = 8
        SET_CONFIGURATION = 9
        GET_INTERFACE = 10
        SET_INTERFACE = 11
        SYNCH_FRAME = 12
     */

}
