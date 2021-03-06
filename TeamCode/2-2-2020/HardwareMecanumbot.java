package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    ColorSensor color;
    
    // motor declarations
    public DcMotor leftFront  = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear   = null;
    public DcMotor rightRear  = null;
    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;
    public DcMotor liftBlock = null;
    public DcMotor lift = null;
    
    // motor for arm
    public Servo servoArm = null;
    public Servo servoGrab1 = null;
    public Servo servoGrab2 = null;
    public Servo servoTray1 = null;
    public Servo servoTray2 = null;
    
    //variables
    private double leftF;
    private double rightF;
    private double leftR;
    private double rightR;
    private int neg = 1;
    private double forward;
    private double turn;
    private double strafe;
    private boolean grab       = false;
    private boolean toggleGrab = false;
    private boolean grabPrev;
    private double varSneak;
    private boolean suck;
    private boolean unsuck;

    public HardwareMecanumbot(){}
    
    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        myOpMode = opMode;

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
        liftBlock   = myOpMode.hardwareMap.get(DcMotor.class, "liftBlock");
        lift        = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        servoArm  = myOpMode.hardwareMap.get(Servo.class,   "servoArm");
        servoGrab1  = myOpMode.hardwareMap.get(Servo.class,   "servoGrab1");
        servoGrab2  = myOpMode.hardwareMap.get(Servo.class,   "servoGrab2");
        servoTray1  = myOpMode.hardwareMap.get(Servo.class,   "servoTray1");
        servoTray2  = myOpMode.hardwareMap.get(Servo.class,   "servoTray2");
        
        //init imu
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        color = myOpMode.hardwareMap.colorSensor.get("color");
        color.enableLed(false);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        //encoders for driving
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //encoders for other motors
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //brake the motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void moveLateral(double f, double t, double s, double vs) {
            //math for mecanum wheels 'f' = forward, 't' = turn, 's' = strafe
            leftF  = f + t + s;
            rightF = f - t - s;
            leftR  = f + t - s;
            rightR = f - t + s;
            
            //logic for Sneaking 'vs' = variable sneak, 'ts' = toggle sneak
            vs = Range.clip(vs, 0, 0.85);
            leftF  = Range.clip(leftF, -1 + vs, 1 - vs);
            rightF = Range.clip(rightF, -1 + vs, 1 - vs);
            leftR  = Range.clip(leftR, -1 + vs, 1 - vs);
            rightR = Range.clip(rightR, -1 + vs, 1 - vs);
            
            //set power for mecanum wheels
            leftFront.setPower(leftF);
            rightFront.setPower(rightF);
            leftRear.setPower(leftR);
            rightRear.setPower(rightR);
    } /*
    public void servoMove(double degrees) {
        if(degrees < 0) {
            neg = -1;
            degrees *= -1;
        } else
            neg = 1;
            
        double moveTime = degrees/180.0;
        period.reset();
        while(period.time() < moveTime)
            servoArm.setPower(1*neg);
        servoArm.setPower(0);
    } */
    public void lift(boolean up, boolean down) {
        if(up && !down) {
            lift.setTargetPosition(lift.getCurrentPosition() - 100);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            //while(lift.isBusy()){}
        }
        else if(down && !up) {
            lift.setTargetPosition(lift.getCurrentPosition() + 100);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            //while(lift.isBusy()){}
        } else {
            lift.setTargetPosition(lift.getCurrentPosition());
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            //while(lift.isBusy()){}
        }
    }
    public void eatTray(boolean close) {
        if(close) {
            servoTray1.setPosition(0.7);
            servoTray2.setPosition(0.6);
        } else {
            servoTray1.setPosition(1);
            servoTray2.setPosition(0);
        }
    }
    public void liftPos(int pos) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        while(lift.isBusy()){}
    }
    public void intake(boolean succ, boolean yeet) {
        //logic for intake system
        if(succ && !yeet) {
            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);
        } else if(yeet && !succ) {
            intakeLeft.setPower(-0.5);
            intakeRight.setPower(-0.5);
        } else {
            intakeLeft.setPower(0.0);
            intakeRight.setPower(0.0);
        }
    }
    public void encoderState(int a){
        if(a == 0){
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if(a == 1){
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if(a == 2){
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if(a == 3){
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
} 
