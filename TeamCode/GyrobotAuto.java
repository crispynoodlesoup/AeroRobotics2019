package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;

@Autonomous(name="Gyrobot Auto", group="Pushbot")

public class GyrobotAuto extends LinearOpMode {
    HardwareMecanumbot      robot   = new HardwareMecanumbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    
    GyroStuff gyro = new GyroStuff();

    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     BIAS                    = 0.5 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        robot.initDrive(this);
        gyro.initDrive(robot);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(1000); // pause for servos to move

        while (opModeIsActive() && !isStopRequested()) {
            //turnToTarget(0);
            driveStraight(0, 0.2);
            
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.addData("pidcalc", "%.2f", gyro.calcPID(0));
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void turnToTarget(double target) {
        //calculates the PID and moves to target angle
        robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
    }
    public void driveStraight(double target, double speed) {
        robot.moveLateral(speed, -gyro.calcPID(target), 0, 0);
    }
    public void driveWithCorrection() {}
    
}
