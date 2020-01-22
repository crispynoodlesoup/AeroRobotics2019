package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;

/**
 * TeleOp for gyrobot with servo and intake system
 */

@TeleOp(name="Gyrobot2Controller", group="Linear Opmode")
public class Gyrobot2Controller extends LinearOpMode {

    // Declare runtime
    private ElapsedTime runtime = new ElapsedTime();
    
    //access the control hub's instruments
    BNO055IMU imu;
    Orientation angle;
    Acceleration gravity;
    
    GyroStuff gyro = new GyroStuff();
    
    //Declare hardware variables
    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftRear    = null;
    private DcMotor rightRear   = null;
    private DcMotor intakeLeft  = null;
    private DcMotor intakeRight = null;
    private DcMotor lift        = null;
    private Servo   servoArm    = null;
    private Servo   servoGrab1  = null;
    private Servo   servoGrab2  = null;
    
    //init variables for movement
    private double forward;
    private double turn;
    private double strafe;
    private double leftF;
    private double rightF;
    private double leftR;
    private double rightR;
    private boolean grab       = false;
    private boolean toggleGrab = false;
    private boolean grabPrev;
    private double varSneak;
    private boolean succ;
    private boolean yeet;
    private boolean suck;
    private boolean unsuck;
    private double globalAngle;
    private int neg1 = 1;
    private int neg2 = 1;
    private int neg3 = 1;
    private double tan;
    private double poly;
    private double correction;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        lift        = hardwareMap.get(DcMotor.class, "lift");
        servoArm    = hardwareMap.get(Servo.class,   "servoArm");
        servoGrab1  = hardwareMap.get(Servo.class,   "servoGrab1");
        servoGrab2  = hardwareMap.get(Servo.class,   "servoGrab2");
        
        lift.setTargetPosition(lift.getCurrentPosition());
        //set all motors to work with encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //init imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        
        //set arm at initialization position
        servoGrab1.setPosition(1);
        servoGrab2.setPosition(0.6);
        
        //setup telemetry
        setupTelemetry();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for variables to determine mecanum spin
            forward   = -gamepad1.left_stick_y;
            turn      = gamepad1.right_stick_x;
            strafe    = gamepad1.left_stick_x;
            grabPrev  = grab;
            grab      = gamepad2.a;
            varSneak  = gamepad1.right_trigger;
            
            //logic for toggleSneak, if stick is pressed invert
            if(grab && !grabPrev)
                toggleGrab = !toggleGrab;
            
            //all lateral movement
            
            lift(gamepad2.dpad_up, gamepad2.dpad_down);
            
            //code for intake
            suck   = gamepad1.left_bumper;
            unsuck = gamepad1.right_bumper;
            intake(suck, unsuck);
            
            //basically all the code for servo
            if(toggleGrab) {
                servoGrab1.setPosition(0.8);
                servoGrab2.setPosition(0.8);
            } else {
                servoGrab1.setPosition(0.5);
                servoGrab2.setPosition(1);
            }
            
            if(gamepad1.dpad_down)
                moveLateral(forward, calcPID(0), strafe, varSneak);
            else if(gamepad1.dpad_right)
                moveLateral(forward, calcPID(90), strafe, varSneak);
            else if(gamepad1.dpad_up)
                moveLateral(forward, calcPID(180), strafe, varSneak);
            else if(gamepad1.dpad_left)
                moveLateral(forward, calcPID(270), strafe, varSneak);
            else
                moveLateral(forward, turn, strafe, varSneak);
            
            servoArm.setPosition(Range.clip(gamepad1.left_trigger, 0.35, 0.95));
            
            //get global angle
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            globalAngle = (angle.firstAngle+360)%360;
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("lift:", lift.getCurrentPosition());
            telemetry.addData("servoArm:", servoArm.getPosition());
            telemetry.update();
        }
    }
    public void moveLateral(double f, double t, double s, double vs) {
            neg1 = 1;
            neg2 = 1;
            neg3 = 1;
        
            //code for calculating controller correction values
            if(s < 0)
                neg2 = -1;
            if(f < 0)
                neg1 = -1;
            if(s == 0) {
                s = 0.0000001;
            }
            if(f == 0) {
                f = 0.0000001;
            }
        
            if(Math.abs(s) > Math.abs(f))
               tan = f/s;
            else
                tan = s/f;
            poly  = neg2*neg1/(1/tan + 1*neg1*neg2);
            if(poly < 0)
                neg3 = -1;
            correction = Math.sqrt((Math.pow((1*neg3-poly*neg3), 2) + Math.pow(poly, 2))/(Math.pow(tan, 2) + 1));
            f *= correction;
            s *= correction;
            correction = Math.abs(f+s);
            f *= correction;
            s *= correction;
        
            //math for mecanum wheels 'f' = forward, 't' = turn, 's' = strafe
            leftF  = f + t + s;
            rightF = f - t - s;
            leftR  = f + t - s;
            rightR = f - t + s;
            
            //logic for Sneaking 'vs' = variable sneak
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
    }
    public void lift(boolean up, boolean down) {
        if(up && !down) {
            lift.setTargetPosition(lift.getCurrentPosition() - 1000);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            //while(lift.isBusy()){}
        }
        else if(down && !up) {
            lift.setTargetPosition(lift.getCurrentPosition() + 1000);
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
    public void liftPos(int pos) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
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
    public void setupTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angle   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                }
            });
            
        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angle.angleUnit, angle.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angle.angleUnit, angle.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angle.angleUnit, angle.thirdAngle);
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public double calcPID(double target) {
        double angleError = target - getAngle();
        if(Math.abs(angleError) > 0.5)
            return Range.clip(angleError/120.0, -0.66, 0.66)*1.5;
        return 0;
    }
    void resetAngle() {
        //myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    //reading angle objects z axis
    public double getAngle() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }
}
