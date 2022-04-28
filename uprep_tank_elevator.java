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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name="UPrep Tank Elevator", group="Pushbot")

public class uprep_tank_elevator extends LinearOpMode {

    /* Declare OpMode members. */
    PantherBot robot   = new PantherBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     armTime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double MIN_CLAW = 0.0;
    static final double MAX_CLAW = 1.0;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, runtime, PantherBot.TEAMS.upprepElevator);
        armTime.startTime();

        robot.setLeftDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRightDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setLeftDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setRightDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        while(false == gamepad1.back && opModeIsActive()) {
            robot.driveRightMotor(-gamepad1.right_stick_y * 0.9);
            robot.driveLeftMotor(-gamepad1.left_stick_y * 0.9);

            //Carousel checks
            if(gamepad2.x)
            {
                robot.carouselTurn("CCW");
            } else if (gamepad2.b){
              robot.  carouselTurn("CW");
            }else {
                robot.carouselTurn ("stopstring");
            }

            //arm
            if(gamepad2.left_stick_y != 0)
            {
                robot.driveArm(gamepad2.left_stick_y);
            }else {
                robot.driveArm(0);
            }

            //claw
            if(gamepad2.right_trigger > 0) {
                robot.openClaw();
                telemetry.addData("servo", "open");
            }
            else if(gamepad2.left_trigger > 0) {
                robot.closeClaw();
                telemetry.addData("servo", "closed");
            }
            telemetry.addData("servo position", "position: %f", robot.getServoPosition());

            telemetry.update();
        }
        sleep(1000);     // pause for servos to move

        telemetry.addData("Operation", "Complete");
        telemetry.update();
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



    /**
     * This method should be used for turning the motors in opposite directions.
     * @param turn
     */
    private void turningInPlace(float turn)
    {
        //1. Check which direction to turn.
        if(turn > 0){
            //2. Turn left when appropriate.
            driveRightMotor(turn);
           driveLeftMotor(-turn);
        }else if(turn < 0){
        //3. Turn right when appropriate.
            driveRightMotor(turn);
            driveLeftMotor(-turn);
        }else{
                //4. Stop motors if no turning is occurring.
            driveRightMotor(0);
            driveLeftMotor(0);
        }
    }

    /**
     * Drive dual motors for going forward, backward, and turning while doing so.
     * @param turn The turning value from -1 to 1. 1 to turn right and -1 to turn left.
     * @param speed The speed from 0 to 1.
     */
    private void drivingControl(float turn, float speed) {
        if(turn > 0){
            //turning left
            driveDualMotors(calculateTurningSpeed(speed, turn), speed);
            telemetry.addData("left","left %f turn", calculateTurningSpeed(speed, turn));
        } else if (turn < 0){
            telemetry.addData("right","right %f turn", calculateTurningSpeed(speed, turn));
            // Turning right
            driveDualMotors(speed,calculateTurningSpeed(speed, turn));
        } else {
            //driving straight
            driveDualMotors(speed, speed);
        }
    }

    /**
     * Calculate the turning passed based on the speed and the movement of the joystick
     * @param speed Speed from 0 to 1
     * @param turn Turning from -1 to 1
     * @return The speed of the motor based on the floored difference of the speed and turn
     */
    public double calculateTurningSpeed(double speed, double turn)
    {
        double retr = speed - Math.abs(turn);
        if(retr < 0)
            return 0;
        else
            return retr;
    }

    /**
     *  Method to drive two motors with given speed. Does not use the encoder.
     *  @param rightSpeed Speed of the right motor
     *  @param leftSpeed Speed of the left motor
     */
    public void driveDualMotors(double leftSpeed, double rightSpeed) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.setLeftDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setRightDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            driveLeftMotor(leftSpeed);
            driveRightMotor(rightSpeed);
        }
    }

    /**
     * Drive only the right motor with the given speed
     * @param speed [-1,1]
     */
    private void driveRightMotor(double speed)
    {
        robot.driveRightMotor(speed);
    }

    /**
     * Drive only the left motor with the given speed
     * @param speed [-1,1]
     */
    private void driveLeftMotor(double speed)
    {
        robot.driveLeftMotor(speed);
    }

}
