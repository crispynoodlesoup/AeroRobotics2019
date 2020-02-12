package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
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

@Autonomous(name="TrayAuto_Red", group="generated")
public class TrayAuto_Red extends LinearOpMode {
        //conversions for inches, straifConv since straifing is a tiny bit wonky
        int conversion = 88;
        int straifConv = 89;
        int r; 
        
        Boolean exit = false;
        double redSide = 1.0;
        ElapsedTime runtime = new ElapsedTime();
        GyroStuff gyro = new GyroStuff();
        HardwareMecanumbot robot = new HardwareMecanumbot();
        @Override
        public void runOpMode(){
                robot.initDrive(this);
                gyro.initDrive(robot);
                
                //initialize servos
                robot.servoGrab1.setPosition(1);
                robot.servoGrab2.setPosition(0.6);
                robot.servoArm.setPosition(0.32);
                robot.eatTray(false);
                
                //wait and execute
                waitForStart();
                /*
                strafeToPosition(9*redSide, 0.5, 0);
                moveToPosition(32.0, 0.6, 0);
                sleep(800);
                robot.eatTray(true);
                sleep(800);
                driveArc(-90*redSide, -0.85, 0.4*redSide);
                robot.eatTray(false);
                sleep(800);
                moveToPosition(-1.5, 1.0, -90);
                robot.eatTray(true);
                sleep(800);
                moveToPosition(16.0, 1.0, -90);
                moveToPosition(-4.0, 1.0, -90);
                robot.liftPos(-3000);
                turnToTarget(179);
                */
                
                strafeToPosition(18*redSide, 1, 0);
                moveToPosition(30.5, 1, 0);
                robot.eatTray(true);
                sleep(800);
                moveToPosition(-18, 1.0, 0);
                turnToTarget(175);
                robot.eatTray(false);
                sleep(800);
                moveToPosition(-2.5, 1.0, 179.99);
                robot.eatTray(true);
                robot.liftPos(-8000);
                moveToPosition(2.0, 1.0, 179.99);
                }
        public void moveToPosition(double inches, double speed, double target){
                int move  = (int)(Math.round(inches * conversion)) + 30;
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                int decelInches = (int)Math.abs(Math.round(move * 18/26));
                decelInches = (int)Range.clip(decelInches, 0, 1800);
                double speedTemp = speed;
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
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 800) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                speed = Range.clip(speed, 0.1, 1.0);
                driveStraight(target,speed,r);
                //
                //while (((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 || end*r - robot.leftFront.getCurrentPosition()*r > 90) && !isStopRequested()){
                //while ((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 && !isStopRequested()){
                //while (end*r - robot.leftFront.getCurrentPosition()*r > 30 && !isStopRequested()){
                while (robot.leftRear.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy() || robot.rightRear.isBusy()){
                        speed = speedTemp;
                        if(Math.abs(robot.leftFront.getCurrentPosition()*r - start*r) < 800) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                        if(Math.abs(end*r - robot.leftFront.getCurrentPosition()*r) < decelInches) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/1800.0;
                        speed = Range.clip(speed, 0.1, 1.0);
                        driveStraight(target,speed,r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("leftF", "%.3f", robot.leftF);
                        telemetry.addData("rightF", "%.3f", robot.rightF);
                        telemetry.addData("leftR", "%.3f", robot.leftR);
                        telemetry.addData("rightR", "%.3f", robot.rightR);
                        telemetry.addData("end", "%d", (end*r - robot.leftFront.getCurrentPosition()*r));
                        telemetry.update();
                }
                turnToTarget(target);
        }
        public void strafeToPosition(double inches, double speed, double target){
                //
                int move = (int)(Math.round(inches * straifConv));
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
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
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 100) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/100.0;
                speed = Range.clip(speed, 0.1, 1);
                straifStraight(target, speed, r);
                //
                while (robot.leftRear.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy() || robot.rightRear.isBusy()){
                        if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 100) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/100.0;
                        if(Math.abs(end - robot.leftFront.getCurrentPosition()) < 1500) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/1600.0;
                        speed = Range.clip(speed, 0.1, 1);
                        straifStraight(target, speed, r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.update();
                }
                turnToTarget(target);
        }
        public void turnToTarget(double target) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                for(int i = 0; i < 2; i++) {
                        robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                        while (gyro.calcPID(target) != 0 && !isStopRequested()){
                                robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                                telemetry.addData("pidcalc", "%.3f", gyro.calcPID(target));
                                telemetry.addData("leftF", "%.3f", robot.leftF);
                                telemetry.addData("rightF", "%.3f", robot.rightF);
                                telemetry.addData("leftR", "%.3f", robot.leftR);
                                telemetry.addData("rightR", "%.3f", robot.rightR);
                                telemetry.update();
                        }
                        sleep(30);
                }
        }
        public void driveArc(double target, double speed, double turnSpeed) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                while(Math.abs(gyro.calcPID(target)) > 0.2 && !isStopRequested()) {
                        robot.moveLateral(speed, turnSpeed*2, 0, 0);
                }
                turnToTarget(target);
        }
        public void driveStraight(double target, double speed, int reverse) {
                robot.moveLateral(speed, -gyro.calcPID(target)*reverse, 0, 0);
        }
        public void straifStraight(double target, double speed, int reverse) {
                robot.moveLateral(0, gyro.calcPID(target)*reverse, speed, 0);
        }
}
