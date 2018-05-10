#ifndef _Kalman_h
#define _Kalman_h

class Kalman 
{

public:
    Kalman()
    {

        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;

        angle = 0;
        bias = 0; 

        P[0][0] = 0;
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 0;
    };

    double getAngle(double new_angle, double new_rate, double dt)
    {
        rate = new_rate - bias;
        angle += dt * rate;
		
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        y = new_angle - angle;

        S = P[0][0] + R_measure;

        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        angle += K[0] * y;
        bias += K[1] * y;

        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];

        return angle;
    };
    
    void 	setAngle	(double new_angle) 		{ angle = new_angle; };
    double 	getRate		() 				{ return rate; };
    void 	setQangle	(double newQ_angle) 		{ Q_angle = newQ_angle; };
    void 	setQbias	(double newQ_bias) 		{ Q_bias = newQ_bias; };
    void 	setRmeasure	(double newR_measure)		{ R_measure = newR_measure; };
    double 	getQangle	() 				{ return Q_angle; };
    double 	getQbias	() 				{ return Q_bias; };
    double 	getRmeasure	() 				{ return R_measure; };

private:

    double Q_angle; 
    double Q_bias; 
    double R_measure;

    double angle;
    double bias;
    double rate;

    double P[2][2]; 
    double K[2];
    double y;
    double S;
};

#endif
