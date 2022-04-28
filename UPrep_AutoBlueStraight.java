/* Copyright (c) 2017 FIRST. All rights reserved.

 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name="Uprep_AutoBlueStraight", group="Pushbot")

public class UPrep_AutoBlueStraight extends LinearOpMode {

    /* Declare OpMode members. */
    PantherBot robot   = new PantherBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double forward =  1;
    private static final double stop =  0;
    private static final double backward = -1;

    private void runAuto(){

        robot.carouselTurn("stop");
        //Open the claw


//        driveTime(3750, forward, forward);
        encoderDrive(0.2, 10, 10);
        encoderDrive(.4, -10, 10);
        stop();
//        robot.controlLeftClaw(150.0, 0, 280);
//
//        driveForward(1);
//        robot.carouselTurn("CW"); //spin carousel
//        driveForward(-2);
//        robot.turning(-90);
//        driveForward(18);
//        robot.turning(-90);
//        driveForward(108);


//        robot.turning(-90); //turn left in place to carousel
//        driveForward(16.5); // drive to carousel
//        sleep((2000)); // wait 2 seconds
//       driveForward(-.5); //drive backwards half an inch for space to turn
//        robot.turning(90); //turn right in place
//        driveForward(108); //parked

//        driveForward(distance)
    }

    /**
     * Display the direction of motion given speed
     * @param speed [-1,1]
     */
    private void displaySpeed(float speed)
    {
        //Display the value of speed
        telemetry.addData("Speed", "%f", speed);

        //Based on the given value of speed, display the direction of motion
        if(speed == 0)
            telemetry.addData("Direction", "stopped");
        else if (speed > 0)
            telemetry.addData("Speed", "forward");
        else
            telemetry.addData("Speed", "reverse");
    }

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, runtime, PantherBot.TEAMS.uprep);


        robot.setLeftDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRightDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setLeftDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRightDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        while(false == gamepad1.back && opModeIsActive()) {
            runAuto();
            telemetry.update();
        }
        sleep(1000);     // pause for servos to move

        telemetry.addData("Operation", "Complete");
        telemetry.update();
    }

    public void driveForward(double distance)
    {
        encoderDrive(robot.DRIVE_SPEED, distance, distance);
    }


    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.encoderDrive(speed, leftInches, rightInches, COUNTS_PER_INCH);
        }

    }
    /**
     * Drive given the time to drive. Stops for op ending
     * @param time milliseconds to drive
     * @param direction Direction to drive
     */
    public void driveTime(double time, double direction)
    {
        double startTime = runtime.milliseconds();
        while(opModeIsActive() && runtime.milliseconds() < startTime + time) {
            telemetry.addData("time", "%s milliseconds", runtime.milliseconds());
            telemetry.update();
            robot.driveRightMotor(robot.DRIVE_SPEED * direction);
            robot.driveLeftMotor(robot.DRIVE_SPEED * direction);
        }
        robot.stop();
    }
    /**
     * Drive given the time to drive. Stops for op ending
     * @note 2250 time for 90 degrees
     * @param time milliseconds to drive
     * @param right direction for right (-1 for reverse, 1 for forward)
     * @param left direction for left (-1 for reverse, 1 for forward)
     */
    private void driveTime(double time, double left, double right)
    {
        double startTime = runtime.milliseconds();
        while(opModeIsActive() && runtime.milliseconds() < startTime + time) {
            telemetry.addData("time", "%s milliseconds", runtime.milliseconds());
            telemetry.update();
            robot.driveRightMotor(robot.TURN_SPEED * right);
            robot.driveLeftMotor(robot.TURN_SPEED * left);
        }
        robot.stop();
    }
}
