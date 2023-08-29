#ifndef PA10KEISAN_H_
#define PA10KEISAN_H_

#include <math.h>

#define	DEBUG	1

#define HEURISTICS 1

#define NOOK	0
#define	OK		1

#define	V_PREC	%18.10f

#define	INCORRECT_ANGLE		-254
#define UNREACH_POSITION	-253

#define PI		3.14159265358979323846264338327950288

#define DEG2RAD(x)		(x*M_PI)/180.0
#define RAD2DEG(x)		(x*180.0)/M_PI

// Using meters instead of millimeters significantly reduces the error.
#define LBs		0.317
#define LB		0.0
#define	LS		0.450
#define LE		0.480
#define LW		0.070

#define	C(x)	cos(x)
#define S(x)	sin(x)

#define	C1		C(j[1])
#define C2		C(j[2])
#define C3		C(j[3])
#define C4		C(j[4])
#define C5		C(j[5])
#define C6		C(j[6])
#define C7		C(j[7])
#define C23		C(j[2]+j[3])

#define	S1		S(j[1])
#define S2		S(j[2])
#define S3		S(j[3])
#define S4		S(j[4])
#define S5		S(j[5])
#define S6		S(j[6])
#define S7		S(j[7])
#define S23		S(j[2]+j[3])





int forward_kinematics(double t[5][5],double j[7+1]);
int inverse_kinematics(double j[7+1],double t[5][5]);

#endif /*PA10KEISAN_H_*/
