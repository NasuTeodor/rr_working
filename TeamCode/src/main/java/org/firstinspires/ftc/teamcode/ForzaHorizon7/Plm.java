package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class Plm {
    public double nr;
    private double nr2;

    public Plm(double nr, double nr2){
        this.nr = nr;
        this.nr2 = nr2;
    }

    public double getNr(){
        return  this.nr;
    }

}
