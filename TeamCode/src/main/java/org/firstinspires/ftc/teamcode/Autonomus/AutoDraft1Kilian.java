//package org.firstinspires.ftc.teamcode.Autonomus;
//
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name = "autos: k ", group = "k")
//public class AutoDraft1Kilian extends LinearOpMode {
//
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
//
//    // arm motors
//    private DcMotor left_Arm_Base_Segment, right_Arm_Base_Segment = null;
//    //naming depends on actual structure, i try to have names that describe the intended function
//
//    private Servo left_intake_servo, right_intake_servo = null;
//    ///depends on structure
//
//    private DcMotor left_Odo = null;
//    private DcMotor right_Odo = null;
//    private DcMotor back_Odo = null;
//
//
//
//
//    @Override
//    public void runOpMode(){
//
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_Left_drive");//fix configuration to --"_left_"
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
//
////        //odometry
////        left_Odo = hardwareMap.get(DcMotor.class, "left_odo");
////        right_Odo = hardwareMap.get(DcMotor.class, "right_odo");
////        back_Odo = hardwareMap.get(DcMotor.class, "back_odo");
////        //odometry
//
//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);// might be wrong direction
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//
//
//        left_Odo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_Odo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        back_Odo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
//       left_Odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       back_Odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       right_Odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        //
//
//        while (opModeInInit()){
//            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
//            telemetry.update();
//        }
//
//    }
//
//
//
//
//
//
//
//}
