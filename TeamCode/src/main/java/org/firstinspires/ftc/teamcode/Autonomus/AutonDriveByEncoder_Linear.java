///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.Autonomus;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
///*
// * This OpMode illustrates the concept of driving a path based on encoder counts.
// * The code is structured as a LinearOpMode
// *
// * The code REQUIRES that you DO have encoders on the wheels,
// *   otherwise you would use: RobotAutoDriveByTime;
// *
// *  This code ALSO requires that the drive Motors have been configured such that a positive
// *  power command moves them forward, and causes the encoders to count UP.
// *
// *   The desired path in this example is:
// *   - Drive forward for 48 inches
// *   - Spin right for 12 Inches
// *   - Drive Backward for 24 inches
// *   - Stop and close the claw.
// *
// *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
// *  that performs the actual movement.
// *  This method assumes that each movement is relative to the last stopping place.
// *  There are other ways to perform encoder based moves, but this method is probably the simplest.
// *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//
//@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
//@Disabled
//public class AutonDriveByEncoder_Linear extends LinearOpMode {
//
//    /* Declare OpMode members. */
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
//    private double left_odo_pos = 0;/////////////////////////////
//    private double right_odo_pos = 0;/////////////////////////////
//    private double back_odo_pos = 0;/////////////////////////////
//
//    // Calculate the COUNTS_PER_INCH for your specific drive train.
//    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
//    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
//    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
//    // This is gearing DOWN for less speed and more torque.
//    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//    static final double     COUNTS_PER_MOTOR_REV    = 8192 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double WHEEL_DIAMETER_INCHES_drive = 4.0 ;// For figuring circumference
//
//    static final double ODO_WHEEL_DIAMETER = 1.25984 ;// For figuring circumference
//    static final double ODO_TICKS_PER_REV = 1.25984 ;// For figuring circumference
//
//
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES_drive * 3.1415);
//    static final double     DRIVE_SPEED             = 0.4;
//    static final double     TURN_SPEED              = 0.5;
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize the drive system variables.
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "back_Left_drive");
//
//
//        left_Odo = hardwareMap.get(DcMotor.class,"left_Odo");//port 1
//        right_Odo = hardwareMap.get(DcMotor.class,"right_Odo");// port 3
//        back_Odo = hardwareMap.get(DcMotor.class,"back_Odo");//port 0
//
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);// might be wrong direction
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        ///left_Odo.setMode(DcMotor.);
//
//
//        left_Odo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_Odo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// //////////////////////////////////////////////////////////
//        left_odo_pos = left_Odo.getCurrentPosition();
//        right_odo_pos = right_Odo.getCurrentPosition();
//        back_odo_pos = back_Odo.getCurrentPosition();
//
//
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Starting at",  "%7d :%7d",
//                          leftFrontDrive.getCurrentPosition(),
//                          rightFrontDrive.getCurrentPosition());
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // Step through each leg of the path,
//        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);  // pause to display final telemetry message.
//    }
//
//    /*
//     *  Method to perform a relative move, based on encoder counts.
//     *  Encoders are not reset as the move is based on the current position.
//     *  Move will stop if any of three conditions occur:
//     *  1) Move gets to the desired position
//     *  2) Move runs out of time
//     *  3) Driver stops the OpMode running.
//     */
//    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
//        int newLeftTargetFRONT;
//        int newLeftTargetBACK;
//        int newRightTargetBACK;
//        int newRightTargetFRONT;
//        int newBackTargetODOPOS;
//
//        // Ensure that the OpMode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//               newLeftTargetFRONT = left_Odo.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//               //newLeftTargetBACK = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//
//               newRightTargetBACK = right_Odo.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            newBackTargetODOPOS = back_Odo.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            //4 independent drive wheels
//               leftFrontDrive.setTargetPosition(newLeftTargetFRONT);
//               leftBackDrive.setTargetPosition(newLeftTargetFRONT);
//               rightFrontDrive.setTargetPosition(newRightTargetFRONT);
//               rightBackDrive.setTargetPosition(newRightTargetBACK);
//
//            // Turn On RUN_TO_POSITION
//            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            //
//            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            leftFrontDrive.setPower(Math.abs(speed));
//            rightFrontDrive.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                   (runtime.seconds() < timeoutS) &&
//                   (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d", newLeftTargetFRONT,  newRightTargetFRONT);
//                telemetry.addData("Currently at",  " at %7d :%7d", left_Odo.getCurrentPosition(), right_Odo.getCurrentPosition(), back_Odo.getCurrentPosition());
//                //telemetry.addData("left odo wheel pos:  ", )
//                /*
//                                telemetry.addData("Running to",  " %7d :%7d", newLeftTargetFRONT,  newRightTargetFRONT);
//
//                                telemetry.addData() returns a non primative data type, (a data type created by ftc sdk dev)
//                                as far as i know you cant directly assasign odo_wheel_pos = telemetry.addData(odo_wheel.getCurrentPosition()
//                                bc telemtry data type is for screen display only.
//
//                                to use tel
//
//                 */
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            leftFrontDrive.setPower(0);
//            rightFrontDrive.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            sleep(250);   // optional pause after each move.
//        }
//    }
//}
