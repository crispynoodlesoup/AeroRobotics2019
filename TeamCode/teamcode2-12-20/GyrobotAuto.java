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
    
    //width = 18.0; //inches
    //cpr = 383.6; //counts per rotation
    //gearratio = 13.7;
    //diameter = 3.937;
    //cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, cpr * gear ratio / (2 * pi * diameter)
    //bias = 0.205;
    Double conversion = 87.12;
    
    // meccyBias = 0.21;//change to adjust only strafing movement
    Double straifConv = 89.245;

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
            turnToTarget(179.999);
            //driveStraight(0, 0.2);
            
            telemetry.addData("pidcalc", "%.2f", gyro.calcPID(-90));
            telemetry.addData("error", "%.3f", gyro.getError());
                        telemetry.addData("leftF", "%.3f", robot.leftF);
                        telemetry.addData("rightF", "%.3f", robot.rightF);
                        telemetry.addData("leftR", "%.3f", robot.leftR);
                        telemetry.addData("rightR", "%.3f", robot.rightR);
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