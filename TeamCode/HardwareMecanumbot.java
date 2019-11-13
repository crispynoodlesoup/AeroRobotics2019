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

public class HardwareTestbot
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

    public HardwareTestbot(){
    }
    
    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        myOpMode = opMode;

        //encoderState("off");
        encoderState("run");
        encoderState("reset");

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
        leftFront   = hardwareMap.get(DcMotor.class, "left_front");
        rightFront  = hardwareMap.get(DcMotor.class, "right_front");
        leftRear    = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear   = hardwareMap.get(DcMotor.class, "right_rear");
        intakeLeft  = hardwareMap.get(DcMotor.class, "intake_left");
        intakeRight = hardwareMap.get(DcMotor.class, "intake_right");
        servoArm    = hardwareMap.get(Servo.class,   "servoArm");
        
        //init imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        
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
            FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else if( a.equals("run")){
            FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(a.equals("position")){
            FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else if(a.equals("off")){
            FLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}   
