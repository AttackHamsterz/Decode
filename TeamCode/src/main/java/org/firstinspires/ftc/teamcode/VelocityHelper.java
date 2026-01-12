package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Collections;
import java.util.Vector;

/**
 * This class accumulates velocities and smooths them over a certain window of measurements
 */
public class VelocityHelper {
    private int numAverage;
    private double velocityXTotal;
    private double velocityYTotal;
    private int index;
    private final Vector<Double> VelX;
    private final Vector<Double> VelY;
    private Position lastPosition;
    private long lastTimeMS;

    /**
     * Velocity helper constructor
     * @param numAverage The samples to average over for velocity
     */
    public VelocityHelper(int numAverage) {
        this.numAverage = 0;
        velocityXTotal = 0;
        velocityYTotal = 0;
        index = 0;
        VelX = new Vector<>(Collections.nCopies(numAverage, 0.0));
        VelY = new Vector<>(Collections.nCopies(numAverage, 0.0));
        lastPosition = null;
        lastTimeMS = 0;
    }

    /**
     * The average x velocity over numAverage
     * @return average x velocity
     */
    public double getVx() {
        return (numAverage <= 0) ? 0 : velocityXTotal/(double)(numAverage);
    }

    /**
     * The average y velocity over numAverage
     * @return average y velocity
     */
    public double getVy() {
        return (numAverage <= 0) ? 0 : velocityYTotal/(double)(numAverage);
    }

    /**
     * Given a new position, calculate new velocities and save them
     * @param pos position to use in next calculations
     */
    public void addPosition(Position pos) {
        long currentTimeMS = System.currentTimeMillis();
        if (lastPosition != null) {
            double deltaX = lastPosition.x - pos.x;
            double deltaY = lastPosition.y - pos.y;
            double deltaT = (double)(currentTimeMS - lastTimeMS)*0.001;
            double VX = deltaX/deltaT;
            double VY = deltaY/deltaT;

            velocityXTotal-=VelX.get(index);
            velocityXTotal+=VX;
            VelX.set(index, VX);

            velocityYTotal-=VelY.get(index);
            velocityYTotal+=VY;
            VelY.set(index, VY);

            index = (index+1)%VelX.size();

            if(numAverage<VelX.size())
                numAverage++;
        }
        lastPosition = pos;
        lastTimeMS = currentTimeMS;
    }

    /**
     * Reset the helper when measurements are stale
     */
    public void reset() {
        Collections.fill(VelX, 0.0);
        Collections.fill(VelY, 0.0);
        numAverage = 0;
        velocityXTotal = 0;
        velocityYTotal = 0;
    }
}
