package org.firstinspires.ftc.teamcode.references;
import com.qualcomm.robotcore.eventloop.opmode.*; import com.qualcomm.robotcore.hardware.*; import com.pedropathing.follower.*; import com.pedropathing.geometry.*; import com.pedropathing.paths.*; import org.firstinspires.ftc.teamcode.pedroPathing.Constants; @Autonomous(name="Auto30")
public class ForFun2 extends OpMode{
    DcMotorEx oL,oR; DcMotor in; Servo s; Follower f; int st=0,sh=0,b=0; boolean sd=false; PathChain[] p=new PathChain[9];
    public void init(){oL=hw("outtakeLeft");oR=hw("outtakeRight");in=hardwareMap.get(DcMotor.class,"intake");s=hardwareMap.get(Servo.class,"servo");
        f= Constants.createFollower(hardwareMap); f.setStartingPose(ps(21.46,122.73,324));
        p[0]=pb(21.46,122.73,53.8,90); p[1]=pc(53.8,90,55.4,83.7,45,84); p[2]=pb(45,84,19.5,84);
        p[3]=pb(19.5,84,53.8,90); p[4]=pc(53.8,90,59,71.5,50,60); p[5]=pb(50,60,24,60);
        p[6]=pb(24,60,53.8,90); p[7]=pb(53.8,90,50,35.5); p[8]=pb(50,35.5,15,36);}
    DcMotorEx hw(String n){return hardwareMap.get(DcMotorEx.class,n);} Pose ps(double x,double y,double h){return new Pose(x,y,Math.toRadians(h));}
    PathChain pb(double x1,double y1,double x2,double y2){return f.pathBuilder().addPath(new BezierLine(new Pose(x1,y1),new Pose(x2,y2))).build();}
    PathChain pc(double x1,double y1,double x2,double y2,double x3,double y3){return f.pathBuilder().addPath(new BezierCurve(new Pose(x1,y1),new Pose(x2,y2),new Pose(x3,y3))).build();}
    void o(int v){oL.setVelocity(-v);oR.setVelocity(v);} void i(double p){in.setPower(p);} void sv(double p){s.setPosition(p);}
    void shoot(){switch(sh){case 0:sv(.12);i(.9);sh=1;break;case 1:if(Math.abs(oR.getVelocity())<1800)sh=2;break;case 2:i(0);b++;if(b>=2){b=0;sh=0;sd=true;}else i(.9);break;}}
    public void loop(){f.update();if(sd){sd=false;o(0);}switch(st){
        case 0:o(1400);f.followPath(p[0]);st++;break;
        case 1:if(!f.isBusy()){sv(0);shoot();if(sd)st++;}break;
        case 2:f.followPath(p[1]);st++;break;
        case 3:if(!f.isBusy()){i(.9);f.followPath(p[2]);st++;}break;
        case 4:if(!f.isBusy()){i(0);o(1400);f.followPath(p[3]);st++;}break;
        case 5:if(!f.isBusy()){shoot();if(sd){f.followPath(p[4]);st++;}}break;
        case 6:if(!f.isBusy()){i(.9);f.followPath(p[5]);st++;}break;
        case 7:if(!f.isBusy()){i(0);o(1400);f.followPath(p[6]);st++;}break;
        case 8:if(!f.isBusy()){shoot();if(sd){f.followPath(p[7]);st++;}}break;
        case 9:if(!f.isBusy()){i(.9);f.followPath(p[8]);st++;}break;
        case 10:if(!f.isBusy())i(0);break;}}
}