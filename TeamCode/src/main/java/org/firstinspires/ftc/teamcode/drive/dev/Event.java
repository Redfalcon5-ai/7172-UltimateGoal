package org.firstinspires.ftc.teamcode.drive.dev;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Event {
    private Pose2d pos;
    private int tag;

    public Event(Pose2d pos, int tag){
        this.pos = pos;
        this.tag = tag;
    }

    public Pose2d getPos(){
        return this.pos;
    }

}
