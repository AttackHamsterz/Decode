package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.Vector;

/**
 * This class accumulates velocities and smooths them over a certain window of measurements
 */
public class DistanceHelper {
    //this stores the eye class
    private int numAverage;
    //not used, but would keep track of the summed y velocities
    private double distanceTotal;
    //summed distances
    private int distIndex;
    //keeps track of the position to add new distance values at

    private final Vector<Double> distances;

    public Telemetry tele = null;


    /**
     * Not in use. Averages the 2D velocities
     * @param numAverage The samples to average over for distance
     */
    public DistanceHelper(int numAverage) {
        this.numAverage = numAverage/2;
        distIndex = 0;
        distances = new Vector<>(Collections.nCopies(numAverage, 0.0));
    }

    /**
     * The average distance over numAverage
     * @return is the average
     */
    public double getAverageDistance(){
        return (numAverage <= 0) ? 0 : distanceTotal / (double)(numAverage);
    }

    /**
     * same thing except we can give it a distance if we want
     * @param distance its the distance lol
     */
    public void addDistance(double distance) {
        distanceTotal -= distances.get(distIndex);
        distanceTotal += distance;
        distances.set(distIndex, distance);
        distIndex = (distIndex + 1) % distances.size();
        if(numAverage<distances.size()) {
            numAverage++;
        }
        if(tele != null) {
            tele.addData("Current Distance", distance);
            tele.addData("Average Distance", getAverageDistance());
            tele.addData("DistanceTotal",distanceTotal);
            tele.addData("numAverage", numAverage);
            tele.addData("Buffer Index", distIndex);
            tele.update();
        }
    }

    /**
     * Reset the helper when measurements are stale
     */
    public void reset() {
        numAverage = 0;
        Collections.fill(distances, 0.0);
        distanceTotal = 0;
    }
}