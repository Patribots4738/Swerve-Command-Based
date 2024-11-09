package frc.robot.util.custom;

/**
 * PID constants used to create PID controllers
 * This class is special becuase of its overloaded constructors
 * It just helps keep everything organized
 */
public class GainConstants {

    // Generic
    private final double P; 
    private final double I;
    private final double D;

    // Spark
    private final double FF;
    private final double iZone;
    private final double minOutput;
    private final double maxOutput;

    // Talon
    private final double S;
    private final double V;
    private final double G;

    // Generic
    public GainConstants(double P, double I, double D) {
        this(P, I, D, 0);
    }

    // Spark
    public GainConstants(double P, double I, double D, double FF) {
        this(P, I, D, FF, -1, 1, true);
    }
    
    // Spark
    public GainConstants(double P, double I, double D, double minOutput, double maxOutput) {
        this(P, I, D, 0, minOutput, maxOutput, true);
    }

    // Talon
    public GainConstants(double P, double I, double D, double S, double V, double G) {
        this(P, I, D, S, V , G, false);
    }

    // Generic
    public GainConstants(double P, double I, double D, double val0, double val1, double val2, boolean isSpark) {
        this(P, I, D, isSpark ? val0 : 0, Double.POSITIVE_INFINITY, isSpark ? val1 : 0, val2, isSpark ? 0 : val0, isSpark ? 0 : val1, isSpark ? 0 : val2);
    }

    // Generic
    public GainConstants(double P, double I, double D, double FF, double iZone, double minOutput, double maxOutput, double S, double V, double G) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.FF = FF;
        this.iZone = iZone;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.S = S;
        this.V = V;
        this.G = G;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public double getFF() {
        return FF;
    }

    public double getIZone() {
        return iZone;
    }

    public double getMinOutput() {
        return minOutput;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public double getS() {
        return S;
    }

    public double getV() {
        return V;
    }

    public double getG() {
        return G;
    }

}