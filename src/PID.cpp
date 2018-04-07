#include "PID.h"
#include <iostream>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->p_error = 0;
	this->d_error = 0;
	this->i_error = 0;

	this->iter_count = 0;
	this->total_square_error = 0;
}

void PID::InitTwiddle(double Tp, double Ti, double Td) {
	this->Tp = Tp;
	this->Ti = Ti;
	this->Td = Td;
}

void PID::UpdateError(double cte) {
	if(this->i_error==0){
		this->d_error=0;
	}else{
		this->d_error = cte - this->p_error;
	}
	this->p_error = cte;
	this->i_error+=cte;

	iter_count++;
	//Ignore first 200 iterations for the car to settle and then consider the error.
	if(iter_count>200) total_square_error+= (cte*cte);
}

double PID::getSteeringAngle() {
	return -Kp*p_error-Kd*d_error-Ki*i_error;
}

long PID::getIterationCount() {
	return iter_count;
}

double PID::getAverageError() {
	return total_square_error/iter_count;
}

double PID::getBestError() {
	return best_error;
}

void PID::setBestError(double error){
	this->best_error = error;
}

void PID::printValues(){
	std::cout<<"Co-Efficients are, P: " <<Kp<< ", I: "<<Ki <<", D: "<<Kd << std::endl;
	std::cout<<"Twiddle params are, P: " <<Tp<< ", I: "<<Ti <<", D: "<<Td << std::endl;
	std::cout<<"Best Error: "<<best_error<<std::endl;
}

void PID::updateTwiddleParams(double error){
	/**
	 *  for i in range(len(p)):
            p[i]+=dp[i];					//Step 0
            robot = make_robot()
            x,y,err = run(robot,p)		//Step 1
            if err < best_err:
                best_err = err
                dp[i]*=1.1
            else:
                p[i]-=2*dp[i]
                robot = make_robot()		//step 2
                x,y,err = run(robot,p)
                if err < best_err:
                    best_err = err
                    dp[i]*=1.1
                else:
                    p[i]+=dp[i]
                    dp[i]*=0.9
	 *
	 */
	switch(step){
	case 0:
		if(i==0) Kp+=Tp;
		if(i==1) Ki+=Ti;
		if(i==2) Kd+=Td;
		Init(Kp, Ki, Kd);
		step=1;
		break;
	case 1:
		if(error<best_error){
			best_error = error;
			if(i==0) Tp*=1.1;
			if(i==1) Ti*=1.1;
			if(i==2) Td*=1.1;

			i++;
			if(i==3) i=0;
			step = 0;
			updateTwiddleParams(error);
			break;
		}else{
			if(i==0) Kp-=2*Tp;
			if(i==1) Ki-=2*Ti;
			if(i==2) Kd-=2*Td;
			Init(Kp, Ki, Kd);
			step = 2;
			break;
		}
	case 2:
		if(error<best_error){
			best_error = error;
			if(i==0) Tp*=1.1;
			if(i==1) Ti*=1.1;
			if(i==2) Td*=1.1;

			i++;
			if(i==3) i=0;
			step = 0;
			updateTwiddleParams(error);
			break;
		}else{
			if(i==0) {
				Kp+=Tp;
				Tp*=0.9;
			}
			if(i==1) {
				Ki+=Ti;
				Ti*=0.9;
			}
			if(i==2) {
				Kd+=Td;
				Td*=0.9;
			}
			i++;
			if(i==3) i=0;
			step = 0;
			updateTwiddleParams(error);
			break;
		}
	}
}
