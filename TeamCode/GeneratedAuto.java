package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="GENERATED-Auto", group="generated")
public class GeneratedAuto extends LinearOpMode {
    //28 * 20 / (2ppi * 4.125)
    Double width = 18.0; //inches
    Double cpr = 383.6; //counts per rotation
    Double gearratio = 13.7;
    Double diameter = 3.937;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;
    Double meccyBias = 0.8;//change to adjust only strafing movement
    Double conversion = cpi * bias;
    Boolean exit = false;
    ElapsedTime runtime = new ElapsedTime();
    GyroStuff gyro = new GyroStuff();
    HardwareMecanumbot robot = new HardwareMecanumbot();
    @Override
    public void runOpMode(){
	robot.initDrive(this);
		gyro.initDrive(robot);
		robot.servoArm.setPosition(0);
                
		waitForStart();
                
		//moveToPosition(10.0, 0.8);
        strafeToPosition(-8.1, 0.8);
		robot.servoArm.setPosition(1);
		sleep(1000);
		strafeToPosition(5.0, 0.8);
		moveToPosition(50.0, 0.9);
		robot.servoArm.setPosition(0);
		sleep(1000);
		moveToPosition(-10.0, 0.8);
		strafeToPosition(-5.0, 0.8);
		robot.servoArm.setPosition(1);
		sleep(1000);
		strafeToPosition(5.0, 0.8);
		moveToPosition(18.0, 0.9);
    }
    public void moveToPosition(double inches, double speed){
        
        int move = (int)(Math.round(inches*conversion));
		

		robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
		robot.rightFront.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
		robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() + move);
		robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                
		robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		
		robot.leftFront.setPower(speed);
		robot.rightFront.setPower(speed);
		robot.leftRear.setPower(speed);
		robot.rightRear.setPower(speed);
        
		while (robot.leftRear.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.rightRear.isBusy()){
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
	}
    public void turnWithGyro(double degrees,double time){
		runtime.reset();
        while(gyro.getError(degrees)>2 || runtime.seconds() > time){
            double spin = gyro.calcPID(degrees);
			
            // normalize all motor speeds so no values exceeds 100%.
            spin = Range.clip(spin, -1, 1);

            // Set drive motor power levels.
            robot.leftFront.setPower(spin);
            robot.rightFront.setPower(spin);
            robot.leftRear.setPower(spin);
            robot.rightRear.setPower(spin);
        }
    }
    public void strafeToPosition(double inches, double speed){
		int move = (int)(Math.round(inches * cpi * meccyBias));
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() - move);
        robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() - move);
        robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.leftRear.setPower(speed);
        robot.rightRear.setPower(speed);

        while (robot.leftRear.isBusy() && robot.rightFront.isBusy() && robot.leftFront.isBusy() && robot.rightRear.isBusy()){}
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        }
        public void turnWithEncoder(double input){
            robot.leftFront.setPower(input);
            robot.rightFront.setPower(input);
            robot.leftRear.setPower(input);
            robot.rightRear.setPower(input);
        }
}
