package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Collections;
import java.util.Vector;

/**
 * This class accumulates velocities and smooths them over a certain window of measurements
 */
public class VelocityHelper {
    public Eye eye = null;
    //this stores the eye class
    private int numAverage;
    //the amount of positions (distances) we are currently storing
    private double velocityXTotal;
    //not used, but would keep track of the summed x velocities
    private double velocityYTotal;
    //not used, but would keep track of the summed y velocities
    private double distanceTotal;
    //summed distances
    private int index;
    //keeps track of the position to add new values at

    private Vector<Double> VelX = null;
    private Vector<Double> VelY = null;

    private final Vector<Double> distances;
    //list holding the distances we are sorting
    private Position lastPosition;
    private long lastTimeMS;
    private static double MIN_DELTA_T = 0.001;

    public Telemetry tele = null;


    /**
     * Not in use. Averages the 2D velocities
     * @param numAverage The samples to average over for distance
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

        distances = null;
    }



    /**
     * a constructor to make the velocity helper class!
     * @param numAverage the samples to average over for distance
     * @param eye our eye class :)
     */
    public VelocityHelper(int numAverage, Eye eye) {
        this.numAverage = 0;
        this.eye = eye;

        distanceTotal = 0;
        index = 0;

        distances = new Vector<>(Collections.nCopies(numAverage, 0.0));

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
     * The average distance over numAverage
     * @return is the average
     */
    public double getAverageDistance(){
        return (numAverage <= 0) ? 0 : distanceTotal / (double)(numAverage);
    }

    public double getCurrentDistance() {
        int previousIndex = (index - 1 + distances.size()) % distances.size();
        return distances.get(previousIndex);
    }

    /**
     * it gets the distace from the eye and stores it using a circular queue averaging buffer (<- mark's fancy word)
     */
    public void addDistance() {
        if (eye == null) {return;}
        double distance = eye.getShotDistance();
        distanceTotal -= distances.get(index);
        distanceTotal += distance;
        distances.set(index, distance);
        index = (index + 1) % distances.size();
        if(numAverage<distances.size()) {
            numAverage++;
        }
        if(tele != null) {
            tele.addData("Current Distance", distance);
            tele.addData("Average Distance", getAverageDistance());
            tele.addData("DistanceTotal",distanceTotal);
            tele.addData("numAverage", numAverage);
            tele.addData("Buffer Index", distances.size());
            tele.addData("Buffer Size", distances.size());
            tele.update();
        }
    }

    /**
     * same thing except we can give it a distance if we want
     * @param distance its the distance lol
     */
    public void addDistance(double distance) {
        if (eye == null) {return;}
        distanceTotal -= distances.get(index);
        distanceTotal += distance;
        distances.set(index, distance);
        index = (index + 1) % distances.size();
        if(numAverage<distances.size()) {
            numAverage++;
        }
        if(tele != null) {
            tele.addData("Current Distance", distance);
            tele.addData("Average Distance", getAverageDistance());
            tele.addData("DistanceTotal",distanceTotal);
            tele.addData("numAverage", numAverage);
            tele.addData("Buffer Index", distances.size());
            tele.addData("Buffer Size", distances.size());
            tele.update();
        }
    }




    /**
     * Given a new position, calculate new velocities and save them
     * @param pos position to use in next calculations
     */

    public void addPosition(Position pos) {
        long currentTimeMS = System.currentTimeMillis();
        double deltaT = (double)(currentTimeMS - lastTimeMS)*0.001;
        if(deltaT < MIN_DELTA_T) return;

        if (lastPosition != null) {
            double deltaX = lastPosition.x - pos.x;
            double deltaY = lastPosition.y - pos.y;
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

            if(tele != null)
            {
                tele.addData("deltaX",deltaX);
                tele.addData("deltaY",deltaY);
                tele.addData("deltaT",deltaT);
                tele.addData("VX",VX);
                tele.addData("VY",VY);
                tele.addData("velocityXTotal",velocityXTotal);
                tele.addData("velocityYTotal",velocityYTotal);
                tele.addData("numAverage",numAverage);
                tele.update();
            }
        }
        lastPosition = pos;
        lastTimeMS = currentTimeMS;
    }



    /**
     * Reset the helper when measurements are stale
     */
    public void reset() {
        //Collections.fill(VelX, 0.0);
        //Collections.fill(VelY, 0.0);
        //velocityXTotal = 0;
        //velocityYTotal = 0;
        numAverage = 0;
        distanceTotal = 0;
        Collections.fill(distances, 0.0);

    }
}
