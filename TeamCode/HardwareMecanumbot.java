package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class HardwareMecanumbot
{
    //define opmode members
    private LinearOpMode myOpMode;
    private ElapsedTime period  = new ElapsedTime();
    
    //access instruments of Hub
    BNO055IMU imu;
    Orientation angle;
    Acceleration gravity;
    
    // motor declarations
    public DcMotor leftFront  = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear   = null;
    public DcMotor rightRear  = null;
    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;
    
    // motor for arm
    public Servo   servoArm = null;
    public DcMotor Arm     = null;
    
    //variables
    private double leftF;
    private double rightF;
    private double leftR;
    private double rightR;

    public HardwareMecanumbot(){
    }
    
    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        myOpMode = opMode;

        //encoderState("off");
        //encoderState("run");
        //encoderState("reset");

        //parameters we're sending to the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftFront   = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        rightFront  = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        leftRear    = myOpMode.hardwareMap.get(DcMotor.class, "left_rear");
        rightRear   = myOpMode.hardwareMap.get(DcMotor.class, "right_rear");
        intakeLeft  = myOpMode.hardwareMap.get(DcMotor.class, "intake_left");
        intakeRight = myOpMode.hardwareMap.get(DcMotor.class, "intake_right");
        servoArm    = myOpMode.hardwareMap.get(Servo.class,   "servoArm");
        
        //init imu
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        
        //run using encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //brake the motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void moveLateral(double f, double t, double s, double vs, boolean ts) {
            //math for mecanum wheels 'f' = forward, 't' = turn, 's' = strafe
            leftF  = f + t + s;
            rightF = f - t - s;
            leftR  = f + t - s;
            rightR = f - t + s;
            
            //logic for Sneaking 'vs' = variable sneak, 'ts' = toggle sneak
            vs = Range.clip(vs, 0, 0.85);
            if(ts) {
                leftF  = Range.clip(leftF, -0.4, 0.4);
                rightF = Range.clip(rightF, -0.4, 0.4);
                leftR  = Range.clip(leftR, -0.4, 0.4);
                rightR = Range.clip(rightR, -0.4, 0.4);
            } else {
                leftF  = Range.clip(leftF, -1 + vs, 1 - vs);
                rightF = Range.clip(rightF, -1 + vs, 1 - vs);
                leftR  = Range.clip(leftR, -1 + vs, 1 - vs);
                rightR = Range.clip(rightR, -1 + vs, 1 - vs);
            }
            
            //set power for mecanum wheels
            leftFront.setPower(leftF);
            rightFront.setPower(rightF);
            leftRear.setPower(leftR);
            rightRear.setPower(rightR);
    }
    public void intake(boolean succ, boolean yeet) {
        //logic for intake system
        if(succ && !yeet) {
            intakeLeft.setPower(0.6);
            intakeRight.setPower(0.6);
        } else if(yeet && !succ) {
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);
        } else {
            intakeLeft.setPower(0.0);
            intakeRight.setPower(0.0);
        }
    }
    public void encoderState(String a){
        if(a.equals("reset")){
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else if( a.equals("run")){
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(a.equals("position")){
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else if(a.equals("off")){
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
} 
