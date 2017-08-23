package org.firstinspires.ftc.teamcode;

/**
 * Created by FTC on 12/3/2016.
 */
public interface NeuralNode {
    Link inputs[] = {};
    Link outputs[] = {};

    boolean getOutput();

    Link[] getInputLinks();

    Link[] getOutputLinks();


}
