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

@Autonomous(name="TrueBlue", group="generated")
public class Auto_Blue extends LinearOpMode {
        //width = 18.0; //inches
        //cpr = 383.6; //counts per rotation
        //gearratio = 13.7;
        //diameter = 3.937;
        //cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, cpr * gear ratio / (2 * pi * diameter)
        //bias = 0.205;
        int conversion = 88;
        
        // meccyBias = 0.21;//change to adjust only strafing movement
        int straifConv = 89;
        
        int colorOffset  = 0;
        int colorOffset2 = 0;
        
        int r;
        
        Boolean exit = false;
        double redSide = -1.0;
        ElapsedTime runtime = new ElapsedTime();
        GyroStuff gyro = new GyroStuff();
        HardwareMecanumbot robot = new HardwareMecanumbot();
        @Override
        public void runOpMode(){
                robot.initDrive(this);
                gyro.initDrive(robot);
                
                //robot.liftPos(17000);
                //sleep(500);
                robot.servoGrab1.setPosition(1);
                robot.servoGrab2.setPosition(0.6);
                robot.servoArm.setPosition(0.5);
                //
                waitForStart();
                //
                strafeToPosition(-31.5, 0.95);
                //
                moveToPosition(-4*redSide, 0.5);
                //
                scanColor();
                //
                strafeToPosition(-1.2, 0.5);
                //
                robot.servoArm.setPosition(0.918);
                //
                sleep(400);
                //
                strafeToPosition(10, 0.9);
                //
                moveToPosition(50.5*redSide + colorOffset*redSide, 0.95);
                //
                robot.servoArm.setPosition(0.4);
                //
                sleep(400);
                //
                moveToPosition(-43.0*redSide, 0.95);
                //
                strafeToPosition(-12, 0.8);
                //
                robot.servoArm.setPosition(0.918);
                //
                sleep(400);
                //
                strafeToPosition(12.0, 0.8);
                //
                moveToPosition(44.0*redSide, 0.95);
                //uwu
                robot.servoArm.setPosition(0.4);
                //
                sleep(400);
                //
                moveToPosition(-16.0*redSide, 0.9);
                //
                //strafeToPosition(24, 1.0);
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
                if(inches < 0)
                        r = -1;
                else
                        r = 1;
                //
                driveStraight(0, speed, r);
                //
                while (robot.leftRear.isBusy() || robot.leftFront.isBusy() || robot.rightFront.isBusy() || robot.rightRear.isBusy()){
                        speed *= 0.939 + Math.abs(inches*0.00085);// + (inches * 0.00001);
                        speed = Range.clip(speed, 0.1, 0.999999);
                        driveStraight(0, speed, r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("red", "%d", robot.color.red());
                        telemetry.addData("green", "%d", robot.color.green());
                        telemetry.addData("blue", "%d", robot.color.blue());
                        telemetry.update();
                }
                /*
                while (gyro.calcPID(0) > 0.15){
                robot.moveLateral(0, -gyro.calcPID(0), 0, 0);
                telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("red", "%d", robot.color.red());
                        telemetry.addData("green", "%d", robot.color.green());
                        telemetry.addData("blue", "%d", robot.color.blue());
                telemetry.update();
                }*/
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightRear.setPower(0);
                }
        public void turnWithGyro(double degrees,double time){
                runtime.reset(); 
                /*while(gyro.getError(degrees)>2 || runtime.seconds() > time){
                        double spin = gyro.calcPID(degrees);

                        // normalize all motor speeds so no values exceeds 100%.
                        spin = Range.clip(spin, -1, 1);

                        // Set drive motor power levels.
                        robot.leftFront.setPower(spin);
                        robot.rightFront.setPower(spin);
                        robot.leftRear.setPower(spin);
                        robot.rightRear.setPower(spin);
                }*/
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
                if(inches < 0)
                        r = -1;
                else
                        r = 1;
                //
                straifStraight(0, speed, r);
                //
                while (robot.leftRear.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy() || robot.rightRear.isBusy()){
                        speed *= 0.939 + Math.abs(inches*0.00085);// + (inches * 0.00001);
                        speed = Range.clip(speed, 0.1, 0.999999);
                        straifStraight(0, speed, r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.update();
                }
                /*
                while (gyro.calcPID(0) > 0.15){
                robot.moveLateral(0, -gyro.calcPID(0), 0, 0);
                telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                telemetry.update();
                }*/
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
        public void turnToTarget(double target) {
                //calculates the PID and moves to target angle
                robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
        }
        public void driveStraight(double target, double speed, int reverse) {
                robot.moveLateral(speed, -gyro.calcPID(target)*reverse, 0, 0);
        }
        public void straifStraight(double target, double speed, int reverse) {
                robot.moveLateral(0, -gyro.calcPID(target)*reverse, speed, 0);
        }
        public void scanColor() {
                int cnt = 0;
                robot.color.enableLed(false);
                while(robot.color.red() > 35 && cnt < 2){
                        colorOffset += 8;
                        cnt += 1;
                        moveToPosition(-8*redSide, 0.6);
                        telemetry.addData("combined", "%d", robot.color.argb());
                        telemetry.addData("red", "%d", robot.color.red());
                        telemetry.addData("green", "%d", robot.color.green());
                        telemetry.addData("blue", "%d", robot.color.blue());
                        telemetry.update();
                }
        }
}
