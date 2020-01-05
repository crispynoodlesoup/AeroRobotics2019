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

@Autonomous(name="BlueSideAuto", group="generated")
public class BlueSideAuto extends LinearOpMode {
        //width = 18.0; //inches
        //cpr = 383.6; //counts per rotation
        //gearratio = 13.7;
        //diameter = 3.937;
        //cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, cpr * gear ratio / (2 * pi * diameter)
        //bias = 0.205;
        Double conversion = 87.12;
        
        // meccyBias = 0.21;//change to adjust only strafing movement
        Double straifConv = 89.245;
        
        Boolean exit = false;
        double redSide = -1.0;
        ElapsedTime runtime = new ElapsedTime();
        GyroStuff gyro = new GyroStuff();
        HardwareMecanumbot robot = new HardwareMecanumbot();
        @Override
        public void runOpMode(){
                robot.initDrive(this);
                gyro.initDrive(robot);
                
                waitForStart();
                //
                robot.liftPos(-400);
                
                robot.servoMove(180);
                
                robot.liftPos(60);
                
                robot.servoMove(-90);
                
                strafeToPosition(-28, 0.5);
                //
                moveToPosition(-18*redSide, 0.5);
                //
                robot.servoMove(90);
                //
                sleep(500);
                //
                strafeToPosition(10, 0.5);
                //
                moveToPosition(66.5*redSide, 0.5);
                //
                robot.servoMove(-60);
                //
                sleep(500);
                //
                moveToPosition(-39.0*redSide, 0.5);
                //
                strafeToPosition(-13, 0.5);
                //
                robot.servoMove(50);
                //
                sleep(1000);
                //
                strafeToPosition(13, 0.5);
                //
                moveToPosition(38.0*redSide, 0.5);
                //
                robot.servoMove(-70);
                //
                sleep(500);
                //
                moveToPosition(-14.0*redSide, 0.5);
                }
        public void moveToPosition(double inches, double speed){
                //
                int move = (int)(Math.round(inches * conversion));
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + move);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() + move);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                robot.leftFront.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.leftRear.setPower(speed);
                robot.rightRear.setPower(speed);
                //
                while (robot.leftRear.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.rightRear.isBusy()){
                if (exit) {
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
                }
                }
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
                //
                int move = (int)(Math.round(inches * straifConv));
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() - move);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() - move);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                robot.leftFront.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.leftRear.setPower(speed);
                robot.rightRear.setPower(speed);
                //
                while (robot.leftRear.isBusy() && robot.rightFront.isBusy() && robot.leftFront.isBusy() && robot.rightRear.isBusy()){}
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
        }
        public void liftPos(int pos) {
                robot.lift.setTargetPosition(pos);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
                while(robot.lift.isBusy()){}
        }
        public void turnWithEncoder(double input){
                robot.leftFront.setPower(input);
                robot.rightFront.setPower(input);
                robot.leftRear.setPower(input);
                robot.rightRear.setPower(input);
        }
}
