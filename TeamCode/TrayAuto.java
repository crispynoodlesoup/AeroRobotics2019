package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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
/*
@Autonomous(name="Tray", group="generated")
public class TrayAuto extends LinearOpMode {
        //28 * 20 / (2ppi * 4.125)
        Double width = 18.0; //inches
        Double cpr = 383.6; //counts per rotation
        Double gearratio = 13.7;
        Double diameter = 3.937;
        Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
        Double bias = 0.205;
        Double meccyBias = 0.21;//change to adjust only strafing movement, 0.04576
        Double conversion = cpi * bias;
        Boolean exit = false;
        double servoPos = 0.0;
        ElapsedTime runtime = new ElapsedTime();
        GyroStuff gyro = new GyroStuff();
        HardwareMecanumbot robot = new HardwareMecanumbot();
        @Override
        public void runOpMode(){
                robot.initDrive(this);
                gyro.initDrive(robot);
                
                robot.servoArm.setPosition(0);
                
                waitForStart();
                //
                strafeToPosition(-30.5, 0.35);
                //
                robot.servoArm.setPosition(1);
                sleep(1000);
                servoPos = 1.0;
                //
                strafeToPosition(15, 0.15);
                //
                turnWithEncoder(180, 0.5);
                //
                //robot.servoArm.setPosition(0);
                //sleep(1000);
                //servoPos = 0.0;
                //moveToPosition(-20.0*redSide, 0.5);
                turnWithEncoder(90, 0.5);
                robot.servoArm.setPosition(0.5);
                //
                }
        public void moveToPosition(double inches, double speed){
                //
                int move = (int)(Math.round(inches*conversion));
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
                return;
                }
                }
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
                return;
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
                int move = (int)(Math.round(inches * cpi * meccyBias));
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
                while (robot.leftRear.isBusy() && robot.rightFront.isBusy() && robot.leftFront.isBusy() && robot.rightRear.isBusy()){
                        robot.servoArm.setPosition(servoPos);
                }
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
                return;
        }
        public void turnWithEncoder(double degrees, double input){
                int turn = (int)(Math.round(degrees/19.0 * cpi));
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + turn);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() - turn);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() + turn);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() - turn);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                robot.leftFront.setPower(input);
                robot.rightFront.setPower(input);
                robot.leftRear.setPower(input);
                robot.rightRear.setPower(input);
                
                while (robot.leftRear.isBusy() && robot.rightFront.isBusy() && robot.leftFront.isBusy() && robot.rightRear.isBusy()){
                        robot.servoArm.setPosition(servoPos);
                }
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
                return;
        }
}*/
