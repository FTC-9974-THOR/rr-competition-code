package org.firstinspires.ftc.teamcode.framework;

/**
 * Created by FTC on 6/6/2017.
 *
 * Quaternions represent any rotation in 3d space.
 * Given Euler angles in the form of (ax, ay, az),
 * ax * i + ay * j + az * k
 * where i, j, and k are unit vectors corresponding
 * to the Cartesian axes.
 * A rotation theta around a unit vector
 * u = (ux, uy, uz) = ux * i + uy * j + uz * k
 * can be expressed in quaternion form as
 * q = e^(theta / 2) * (ux * i + uy * j + uz * k) = cos(theta / 2) + (ux * i + uy * j + uz * k) * sin(theta / 2)
 */
public class Quaternion {

    public double x, y, z, w;
}
