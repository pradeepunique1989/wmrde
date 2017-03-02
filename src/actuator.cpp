#include <wmrde/actuator.h>

void PIact( const Real params[], const Real ucmd, const Real u, const Real interr, //inputs
		   Real& f, Real& err, Real& dfdu) { //outputs
	Real Kp = params[0];
	Real Ki = params[1];

	Real fmax = params[2];

	err = ucmd-u;
	f = Kp*err + Ki*interr;
	if (fabs(f) > fmax)
    {
        f=REALSIGN(f)*fmax; //enforce limit
    // printf("\n Hitting saturation in actuator.cpp \n");
    }

    // printf("\n interr : %10.5f , err : %10.5f, ucmd : %10.5f, u : %10.5f\n",interr, err, ucmd,u);

	dfdu = -Kp;
	if (fabs(f) == fmax) dfdu=0.0;

}
