package org.firstinspires.ftc.teamcode.Autonomus;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Marco on 4/13/18.
 */
@Autonomous
public class PracticeAutoOdo {
   // static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    HardwareMap hwMap;
    ElapsedTime clock = new ElapsedTime();
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;
    static final double oneRotationTicks = 800;
    static final double wheelRadius = 0.025; // in meters
    static final double wheelDistanceApart = 0.09144 + .016 * 2.0; // in meters
    private int leftEncoderPos = 0;
    private int centerEncoderPos = 0;
    private int rightEncoderPos = 0;
    private double deltaLeftDistance = 0;
    private double deltaRightDistance = 0;
    private double deltaCenterDistance = 0;
    private double x = 0;
    private double y = 0;
    private double theta = 0;

    /**
     * plug left encoder into frontleft, right encoder into frontright, center encoder into backleft (arbitary assignments)
     */

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // arm motors
    private DcMotor left_Arm_Base_Segment, right_Arm_Base_Segment = null;
    //naming depends on actual structure, i try to have names that describe the intended function

    private Servo left_intake_servo, right_intake_servo = null;
    ///depends on structure

    private DcMotor left_Odo = null;
    private DcMotor right_Odo = null;
    private DcMotor back_Odo = null;
    public void init(HardwareMap hardwareMap, boolean initSensors) {
        hwMap = hardwareMap;
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_Left_drive");//fix configuration to --"_left_"
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        
        //odometry
        left_Odo = hardwareMap.get(DcMotor.class, "left_odo");
        right_Odo = hardwareMap.get(DcMotor.class, "right_odo");
        back_Odo = hardwareMap.get(DcMotor.class, "back_odo");
        //odometry

        //arm
        left_Arm_Base_Segment = hardwareMap.get(DcMotor.class,"left_arm_base");
        right_Arm_Base_Segment = hardwareMap.get(DcMotor.class,"right_arm_base");
        //arm

        //intake Servos
        left_intake_servo = hardwareMap.get(Servo.class, "left_intake_servo");
        right_intake_servo =hardwareMap.get(Servo.class, "right_intake_servo");
        //intake Servos

       
        if (initSensors) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imu.initialize(parameters);
        }
    }
    public void init(HardwareMap ahwMap) {
        init(ahwMap, true);
    }
    public void resetTicks() {
        resetLeftTicks();
        resetCenterTicks();
        resetRightTicks();
    }  
    public void resetLeftTicks() {
        leftEncoderPos = left_Odo.getCurrentPosition();
    }
    public int getLeftTicks() {
        return left_Odo.getCurrentPosition() - leftEncoderPos;
    }
    public void resetRightTicks() {
            rightEncoderPos = right_Odo.getCurrentPosition();
    }
    public int getRightTicks() {
        return right_Odo.getCurrentPosition() - rightEncoderPos;
    }
    public void resetCenterTicks() {
        centerEncoderPos = back_Odo.getCurrentPosition();
    }
    public int getCenterTicks() {
        return back_Odo.getCurrentPosition() - centerEncoderPos;
    }
    public void drive(double fl, double bl, double fr, double br) {
        leftFrontDrive.setPower(fl);
        leftBackDrive.setPower(bl);
        rightFrontDrive.setPower(fr);
        leftBackDrive.setPower(br);
    }
    public void updatePosition() {
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;

        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / wheelDistanceApart;
        resetTicks();
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getTheta() {
        return theta;
    }
    public void setX(double _x) {
        x = _x;
    }
    public void setY(double _y) {
        y = _y;
    }
    public void setTheta(double _theta) {
        theta = _theta;
    }
    public double angle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
}