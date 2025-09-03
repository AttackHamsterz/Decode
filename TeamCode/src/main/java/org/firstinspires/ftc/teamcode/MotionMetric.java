package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class MotionMetric {
    public Pose pose;
    public double power;

    public MotionMetric(Pose pose, double power){
        this.pose = pose;
        this.power = power;
    }
}
