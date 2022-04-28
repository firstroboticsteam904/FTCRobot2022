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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class PantherBot
{
    static enum TEAMS
    {
        museum, uprep, upprepElevator
    }
    private TEAMS currentTeam;
    /* Public OpMode members. */
    private DcMotor  leftDrive     = null;
    private DcMotor  rightDrive    = null;
    private DcMotor carouselDrive = null;
    private DcMotor armDrive      = null;
//    private DcMotor dummyMotor    = null; //example

    private Servo    leftClaw    = null;
    private Servo   rightClaw    = null;
    private ElapsedTime runtime = null;

    public static final double     DRIVE_SPEED             = 0.4;
    public static final double     TURN_SPEED              = 0.1;

                                    //larger means more closed
    public static final double     RIGHT_CLAW_OPEN         = 0.64;
    public static final double     RIGHT_CLAW_START        = 0.15;
    public static final double     RIGHT_CLAW_CLOSED       = 0.65;

                                    //smaller means more closed
    public static final double     LEFT_CLAW_OPEN          = 0.25   ;
    public static final double     LEFT_CLAW_START         = 0.60;
    public static final double     LEFT_CLAW_CLOSED        = 0.05;

    public static final double     CAROUSEL_SPEED        = 0.99;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public PantherBot(){

    }

    public void init(HardwareMap ahwMap, ElapsedTime runtime, TEAMS team) {
        this.runtime = runtime;
        this.init(ahwMap, team);
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, TEAMS team ) {
        if(this.runtime == null)
        {
            this.runtime = new ElapsedTime();
        }

        currentTeam = team;
        // Save reference to Hardware map
        hwMap = ahwMap;

        switch(team)
        {
            case museum: //museum
            {
                setUp_Museum();
                break;
            }
            case uprep: //u prep
            {
                setUp_UPrep();
                break;
            }
            case upprepElevator:
            {
                setUp_UPrepDreamBot();
                break;
            }
            default:
            {
                break;
            }
        }


        // Set all motors to zero power
    }

    private void setUp_Museum()
    {
        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        carouselDrive = hwMap.get(DcMotor.class, "carousel_drive");
        armDrive = hwMap.get(DcMotor.class, "arm_drive");

        leftClaw = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        carouselDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        carouselDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setUp_UPrep()
    {
        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        carouselDrive = hwMap.get(DcMotor.class, "carousel_drive");
        armDrive = hwMap.get(DcMotor.class, "arm_drive");

        leftClaw = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");

        rightClaw.setPosition(RIGHT_CLAW_OPEN);
        leftClaw.setPosition(LEFT_CLAW_OPEN);
        leftDrive.setDirection(DcMotor.Direction.FORWARD); //
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// 
        carouselDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        armDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        carouselDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setUp_UPrepDreamBot()
    {
        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        carouselDrive = hwMap.get(DcMotor.class, "carousel_drive");
        armDrive = hwMap.get(DcMotor.class, "arm_drive");

        leftClaw = hwMap.get(Servo.class, "left_hand"); //Used for spinning the intake wheel

        rightClaw.setPosition(RIGHT_CLAW_OPEN);
        leftClaw.setPosition(LEFT_CLAW_OPEN);
        leftDrive.setDirection(DcMotor.Direction.FORWARD); //
        rightDrive.setDirection(DcMotor.Direction.REVERSE);//
        carouselDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        armDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        carouselDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLeftDriveMode(DcMotor.RunMode mode)
    {
        leftDrive.setMode(mode);
    }
    public void setRightDriveMode(DcMotor.RunMode mode)
    {
        rightDrive.setMode(mode);
    }
    public void setCarouselMode(DcMotor.RunMode mode)
    {
        carouselDrive.setMode(mode);
    }

    @Deprecated
    public void setLeftDrivePower(double power)
    {
        leftDrive.setPower(power);
    }
    @Deprecated
    public void setRightDrivePower(double power)
    {
        rightDrive.setPower(power);
    }
    @Deprecated
    public void setCarouselPower(double power)
    {
        carouselDrive.setPower(power);
    }


    public void carouselTurn(String direction)
    {
        if(direction.equals("CCW") ){
            //turn carousel) motor
            driveCarouselCCW();
        }else if(direction.equals("CW") ){
            //turn carousel) motor
            driveCarouselCW();
        }
        else {
            stopCarousel();
        }
    }

    public void turnIntake(String direction)
    {
        if(direction.equals("CCW"))
            driveIntakeCCW();
        if(direction.equals("CW"))
            driveIntakeCW();
        else{
            stopIntake();
        }
    }

    /**
     * were using A MATH METHOD TO TURN A ROBOT left with positive degrees
     * @param degrees degrees to turn
     */
    public void turning(float degrees ){
        float ratio = ((float)12) / ((float)180);

        float left = degrees * ratio;
        float right = -degrees * ratio;

        this.encoderDrive(DRIVE_SPEED, left, right,5.0 );
    }

    /**
     * This method should be used for turning the motors in opposite directions.
     * @param turn
     */
    public void turningInPlace(float turn)
    {
        turn = turn * (float).75; //decrease the speed by 25%
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
    public void drivingControl(float turn, float speed) {
        if(turn > 0){
            //turning left
            driveDualMotors(calculateTurningSpeed(speed, turn), speed);
//            telemetry.addData("left","left %f turn", calculateTurningSpeed(speed, turn));
        } else if (turn < 0){
//            telemetry.addData("right","right %f turn", calculateTurningSpeed(speed, turn));
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

        this.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reset the timeout time and start motion.
        driveLeftMotor(leftSpeed);
        driveRightMotor(rightSpeed);
    }

    public void encoderDrive(double speed,
                              double leftInches, double rightInches, double countsPerInch)
    {
        // Turn On RUN_TO_POSITION
        this.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine new target position, and pass to motor controller
        int newLeftTarget = this.leftDrive.getCurrentPosition() + (int)(leftInches * countsPerInch);
        int newRightTarget = this.rightDrive.getCurrentPosition() + (int)(rightInches * countsPerInch);

        this.leftDrive.setTargetPosition(newLeftTarget);
        this.rightDrive.setTargetPosition(newRightTarget);


        // reset the timeout time and start motion.
        this.leftDrive.setPower((speed));
        this.rightDrive.setPower((speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((this.leftDrive.isBusy() && this.rightDrive.isBusy())) {
        }
        // Stop all motion;
        this.leftDrive.setPower(0);
        this.rightDrive.setPower(0);


        //  sleep(250);   // optional pause after each move
    }

    /**
     * Drive only the right motor with the given speed
     * @param speed [-1,1]
     */
    public void driveRightMotor(double speed)
    {
        this.rightDrive.setPower(speed);
    }

    /**
     * Drive only the left motor with the given speed
     * @param speed [-1,1]
     */
    public void driveLeftMotor(double speed)
    {
        this.leftDrive.setPower(speed);
    }

    private void driveCarouselCCW()
    {
        this.carouselDrive.setPower(-CAROUSEL_SPEED);
    }

    private void driveCarouselCW()
    {
        this.carouselDrive.setPower(CAROUSEL_SPEED);
    }

    private void stopCarousel()
    {
        this.carouselDrive.setPower(0);
    }

    /**
     * Move the intake servo in the counter clockwise direction.
     * @apiNote Only works for the DreamBot team
     */
    private void driveIntakeCCW()
    {
        if(currentTeam == TEAMS.upprepElevator)
            this.leftClaw.setPosition(0.0);
    }
    /**
     * Move the intake servo in the clockwise direction.
     * @apiNote Only works for the DreamBot team
     */
    private void driveIntakeCW()
    {
        if(currentTeam == TEAMS.upprepElevator)
           this.leftClaw.setPosition(1.0);
    }
    /**
     * Stop the intake servo.
     * @apiNote Only works for the DreamBot team
     */
    private void stopIntake()
    {
        if(currentTeam == TEAMS.upprepElevator)
            this.leftClaw.setPosition(0.5);
    }

    public void stop()
    {
        this.leftDrive.setPower(0);
        this.rightDrive.setPower(0);
        this.armDrive.setPower(0);
    }

    public void driveArm(double speed)
    {
        this.armDrive.setPower(speed/3);
    }

    /**
     * Set position of left claw. Will limit based off max and min given.
     * @param position
     * @param maxposition
     * @param minposition
     */
    public void controlLeftClaw(double position,double maxposition,double minposition )
    {
         if(position >=maxposition){
             leftClaw.setPosition(maxposition);
         }else if(position<=minposition){
             leftClaw.setPosition(minposition);
         }else{
             leftClaw.setPosition(position);
         }
    }

    /**
     *
     * @param position
     * @param max
     * @param min
     */
    public void controlRightClaw(double position, double max, double min)
    {
//        if(position >= max){
//            rightClaw.setPosition(max);
//        }else if(position <= min){
//            rightClaw.setPosition(min);
//        }else{
            rightClaw.setPosition(position);
//        }
    }

    /***
     * Move position of left claw every tick by value of tick.
     * @param tick Value to change position by
     * @param maxposition absolute min is 0
     * @param minoposition absolute max is 280
     */
    public void moveClaw(double tick, double maxposition, double minoposition ){
        final double multiplier = 5; // a multiplier to speed up the servo
        tick = tick * multiplier;
        double position = (leftClaw.getPosition());
        controlLeftClaw(position + tick, maxposition, minoposition);
    }

    public double getServoPosition()
    {
        return leftClaw.getPosition();
    }

    public void openClaw (){
        //Open the claw
        controlRightClaw(RIGHT_CLAW_OPEN,RIGHT_CLAW_OPEN,RIGHT_CLAW_CLOSED);
        controlLeftClaw(LEFT_CLAW_OPEN, LEFT_CLAW_OPEN,LEFT_CLAW_CLOSED);
    }

    public void closeClaw()
    {
        //Open the claw
        controlRightClaw(RIGHT_CLAW_CLOSED,RIGHT_CLAW_OPEN,RIGHT_CLAW_CLOSED);
        controlLeftClaw(LEFT_CLAW_CLOSED, LEFT_CLAW_OPEN,LEFT_CLAW_CLOSED);
    }

    public void startClaw()
    {
        //Open the claw
        controlRightClaw(RIGHT_CLAW_START,RIGHT_CLAW_OPEN,RIGHT_CLAW_CLOSED);
        controlLeftClaw(LEFT_CLAW_START, LEFT_CLAW_OPEN,LEFT_CLAW_CLOSED);
    }

}

