package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@TeleOp(name="Gyrobot", group="Linear Opmode")
public class Gyrobot extends LinearOpMode {

    // Declare runtime
    private ElapsedTime runtime = new ElapsedTime();
    
    //access the control hub's instruments
    BNO055IMU imu;
    Orientation angle;
    Acceleration gravity;
    
    //Declare hardware variables
    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftRear    = null;
    private DcMotor rightRear   = null;
    private DcMotor intakeLeft  = null;
    private DcMotor intakeRight = null;
    private DcMotor lift        = null;
    private Servo   servoArm    = null;
    private Servo   servoGrab   = null;
    
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
        servoGrab   = hardwareMap.get(Servo.class,   "servoGrab");
        
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
        
        //set arm to 0 at initialization
        servoArm.setPosition(0);
        servoGrab.setPosition(0);
        
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
            grabPrev = grab;
            grab     = gamepad1.left_stick_button;
            varSneak  = gamepad1.right_trigger;
            
            //logic for toggleSneak, if stick is pressed invert
            if(grab && !grabPrev)
                toggleGrab = !toggleGrab;
            
            //all lateral movement
            moveLateral(forward, turn, strafe, varSneak);
            
            lift(gamepad1.dpad_up, gamepad1.dpad_down);
            
            //code for intake
            suck   = gamepad1.left_bumper;
            unsuck = gamepad1.right_bumper;
            intake(suck, unsuck);

            //basically all the code for servo lol
            servoArm.setPosition(gamepad1.left_trigger);
            
            if(toggleGrab)
                servoGrab.setPosition(0.9);
            else
                servoGrab.setPosition(1);
            
            //get global angle
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            globalAngle = (angle.firstAngle+360)%360;
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
    public void moveLateral(double f, double t, double s, double vs) {
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
            lift.setTargetPosition(lift.getCurrentPosition() - 25);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            //while(lift.isBusy()){}
        }
        else if(down && !up) {
            lift.setTargetPosition(lift.getCurrentPosition() + 25);
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
}
