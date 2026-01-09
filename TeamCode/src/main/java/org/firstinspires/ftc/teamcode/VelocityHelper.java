package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Collections;
import java.util.Vector;

public class VelocityHelper {
    private int numAverage;
    private double velocityXTotal;
    private double velocityYTotal;
    private int index;
    private Vector<Double> VelX;
    private Vector<Double> VelY;
    private Position lastPosition;
    private long lastTimeMS;


    public VelocityHelper(int numAverage) {
        this.numAverage = numAverage;
        this.velocityXTotal = 0;
        this.velocityYTotal = 0;
        this.index = 0;
        VelX = new Vector<>(numAverage, 0);
        VelY = new Vector<>(numAverage, 0);
        lastPosition = null;
        lastTimeMS = 0;
    }

    public double getVx() {
        return (numAverage <= 0) ? 0 : velocityXTotal/(double)(numAverage);
    }

    public double getVy() {
        return (numAverage <= 0) ? 0 : velocityYTotal/(double)(numAverage);
    }

    public void addPosition(Position pos) {
        long currentTime = System.currentTimeMillis();
        if (lastPosition != null) {
            double deltaX = lastPosition.x - pos.x;
            double deltaY = lastPosition.y - pos.y;
            double deltaT = (double)(currentTime - lastTimeMS)*1e-3;
            double VX = deltaX/deltaT;
            double VY = deltaY/deltaT;

            velocityXTotal-=VelX.get(index);
            velocityXTotal+=VX;
            VelX.add(index, VX);

            velocityYTotal-=VelY.get(index);
            velocityYTotal+=VY;
            VelY.add(index, VY);

            index = (index+1)%numAverage;
        }
        lastPosition = pos;
        lastTimeMS = currentTime;
    }

    public void reset() {
        Collections.fill(VelX, new Double(0));
        Collections.fill(VelY, new Double(0));
        velocityXTotal = 0;
        velocityYTotal = 0;
    }

}
