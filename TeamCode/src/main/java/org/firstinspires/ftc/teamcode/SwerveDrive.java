package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Swerve")
public class SwerveDrive {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private Servo leftFrontServo;
    private Servo leftBackServo;
    private Servo rightFrontServo;
    private Servo rightBackServo;

    Double lF, rF, lB, rB, maxVector;

    static final double SLOMO_DIVIDER = 2.5;

    static final double ROBOT_DIAMETER = 20;

    static final double TICKS_PER_REVOLUTION = 8192;
    static final double WHEEL_DIAMETER = 1.45;
    static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION / (WHEEL_DIAMETER * PI));

    private Gamepad gp;
    private IMU gyro;


    public SwerveDrive(Gamepad gp, DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, Servo lFS, Servo lBS, Servo rFS, Servo rBS, IMU gyro) {

        leftFront = lF;
        leftBack = lB;
        rightFront = rF;
        rightBack = rB;
        leftFrontServo = lFS;
        leftBackServo = lBS;
        rightFrontServo = rFS;
        rightBackServo = rBS;

        this.gp = gp;
        this.gyro = gyro;

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void init() {
        double fwd = -gp.left_stick_y;
        double str = gp.left_stick_x;
        double rcw = gp.right_stick_x;
        double theta = 0.0;//gyro.getRobotOrientation();

        double temp = fwd * Math.cos(theta) + str * Math.sin(theta);
        str =  - fwd * Math.sin(theta) + str * Math.cos(theta);
        fwd = temp;

        double l = 40; //vehicle's wheelbase
        double w = 50; //trackwidth
        double r = sqrt(Math.pow(l,2) + Math.pow(w,2));

        double a = str - rcw * (l/r);
        double b = str + rcw * (l/r);
        double c = fwd - rcw * (w/r);
        double d = fwd + rcw * (w/r);

        // 1=front_right 2=front_left 3=rear_left 4=rear_right

        //wheel speeds
        double ws1 = sqrt(Math.pow(b,2) + Math.pow(c,2));
        double ws2 = sqrt(Math.pow(b,2) + Math.pow(d,2));
        double ws3 = sqrt(Math.pow(a,2) + Math.pow(d,2));
        double ws4 = sqrt(Math.pow(a,2) + Math.pow(c,2));

        //wheel angles
        double wa1 = Math.atan2(b,c) * 180/PI;
        double wa2 = Math.atan2(b,d) * 180/PI;
        double wa3 = Math.atan2(a,d) * 180/PI;
        double wa4 = Math.atan2(a,c) * 180/PI;

        double maxws = ws1;
        if(ws2>maxws)maxws=ws2;
        if(ws3>maxws)maxws=ws3;
        if(ws4>maxws)maxws=ws4;
        if(maxws>1){
            ws1/=maxws;
            ws2/=maxws;
            ws3/=maxws;
            ws4/=maxws;
        }


    }

    public void loop(){

    }

    //Sets the RunMode for all drive motors
    public void setModeAll(DcMotor.RunMode runMode) {

        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightFront.setMode(runMode);
        rightBack.setMode(runMode);

    }


    //sets the ZeroPowerBehavior for all drive motors
    public void setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior zpb) {

        leftFront.setZeroPowerBehavior(zpb);
        leftBack.setZeroPowerBehavior(zpb);
        rightFront.setZeroPowerBehavior(zpb);
        rightBack.setZeroPowerBehavior(zpb);

    }


    //Sets power to all motors
    public void setMotorPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

    }


    //Calculates xComponent for 'goToPosition()' function
    private double calculateX(double desiredAngle, double speed) { return Math.sin(Math.toRadians(desiredAngle)) * speed; }


    //Calculates yComponent for 'goToPosition()' function
    private double calculateY(double desiredAngle, double speed) { return Math.cos(Math.toRadians(desiredAngle)) * speed; }


    //Transform angle to a value between 0 and 360
    public double angleWrapDegrees(double angle) {
        while (angle > 360) {
            angle -= 360;
        } while(angle < 0){
            angle += 360;
        }

        return angle;
    }


    //Transform angle to a value between 0 and 2PI
    public double angleWrapRadians(double angle) {
        while (angle > 2 * PI ) {
            angle -= 2 * PI;
        } while(angle < 0){
            angle += 2 * PI;
        }

        return angle;
    }

}
