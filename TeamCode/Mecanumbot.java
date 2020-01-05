package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp for mechanumbot with CRServo and intake system
 */

@TeleOp(name="Mecanumbot", group="Linear Opmode")
public class Mecanum_Linear extends LinearOpMode {

    // Declare runtime
    private ElapsedTime runtime = new ElapsedTime();
    
    //Declare hardware variables
    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftRear    = null;
    private DcMotor rightRear   = null;
    private DcMotor intakeLeft  = null;
    private DcMotor intakeRight = null;
    private Servo   servoArm    = null;

    @Override
    public void runOpMode() {
        //telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftFront   = hardwareMap.get(DcMotor.class, "left_front");
        rightFront  = hardwareMap.get(DcMotor.class, "right_front");
        leftRear    = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear   = hardwareMap.get(DcMotor.class, "right_rear");
        intakeLeft  = hardwareMap.get(DcMotor.class, "intake_left");
        intakeRight = hardwareMap.get(DcMotor.class, "intake_right");
        servoArm    = hardwareMap.get(Servo.class,   "servoArm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        
        //initialize servo at pos 0
        servoArm.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // initialize variables for sneak
        boolean sneak       = false;
        boolean toggleSneak = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Setup a variable for variables to determine mecanum spin
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            boolean sneakPrev = sneak;
            sneak = gamepad1.left_stick_button;
            double varSneak = gamepad1.right_trigger;

            //math for mecanum wheels
            double leftF = forward + turn + strafe;
            double rightF = forward - turn - strafe;
            double leftR = forward + turn - strafe;
            double rightR = forward - turn + strafe;
        
            //logic for Sneaking
            varSneak = Range.clip(varSneak, 0, 0.85);
            if(sneak && !sneakPrev)
                toggleSneak = !toggleSneak;
            if(toggleSneak) {
                leftF = Range.clip(leftF, -0.4, 0.4);
                rightF = Range.clip(rightF, -0.4, 0.4);
                leftR = Range.clip(leftR, -0.4, 0.4);
                rightR = Range.clip(rightR, -0.4, 0.4);
            } else {
                leftF = Range.clip(leftF, -1 + varSneak, 1 - varSneak);
                rightF = Range.clip(rightF, -1 + varSneak, 1 - varSneak);
                leftR = Range.clip(leftR, -1 + varSneak, 1 - varSneak);
                rightR = Range.clip(rightR, -1 + varSneak, 1 - varSneak);
            }
            
            //set power for mecanum wheels
            leftFront.setPower(leftF);
            rightFront.setPower(rightF);
            leftRear.setPower(leftR);
            rightRear.setPower(rightR);
            
            //variables for intake
            boolean succ = gamepad1.left_bumper;
            boolean unsucc = gamepad1.right_bumper;
            
            //logic for intake system
            if(succ && !unsucc) {
                intakeLeft.setPower(0.5);
                intakeRight.setPower(0.5);
            } else if(unsucc && !succ) {
                intakeLeft.setPower(-0.7);
                intakeRight.setPower(-0.7);
            } else {
                intakeLeft.setPower(0.0);
                intakeRight.setPower(0.0);
            }
            
            //basically all the servo code lol
            servoArm.setPosition(gamepad1.left_trigger);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "leftF (%.2f), rightF (%.2f), leftR (%.2f), rightR (%.2f), intakeLeft (%.2f), intakeRight (%.2f)", leftF, rightF, leftR, rightR, intakeLeft, intakeRight);
            telemetry.update();
        }
    }
}
