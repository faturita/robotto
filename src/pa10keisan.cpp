/**
 *************************************************************************************
 * PLEASE PUT CORRECT HEADER HERE.
 * 
 * みちろん、間違ったことがある。ごめんなさい
 * 
 * 
 ************************************************************************************* 
 *
 * Inverse Kinematics and Forward Kinematics functions for PA 10-7c
 * 
 * TODO:
 * - Add constrains over the angles values (as described in the PA 10 document)
 * - Fix numerical problems 
 * - Run test and perform optimization
 * 
 * 
 *************************************************************************************
 ***/
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>


#include "pa10keisan.h"

double anglescheck[] = {0,177,94,174,137,255,165,255};

//	nx	tx	bx	px
//	ny	ty	by	py
//	nz	tz	bz	pz
//	0	0	0	1
	
#define nx	t[1][1]
#define ny	t[2][1]
#define nz	t[3][1]
#define	tx	t[1][2]
#define ty	t[2][2]
#define tz	t[3][2]
#define bx	t[1][3]
#define by	t[2][3]
#define bz	t[3][3]
#define px	t[1][4]
#define py	t[2][4]
#define pz	t[3][4]

int kakuDoSetsuYaku(int kakuIndex, double degAngle) {
	if (degAngle>=-anglescheck[kakuIndex] && degAngle<=anglescheck[kakuIndex]) {
		return OK;
	} else return NOOK;
}

int checkConstrains(double angle[7+1]) {

	for(int i=1;i<7+1;i++) {
		if (kakuDoSetsuYaku(i,RAD2DEG(angle[i]))!=OK) {
			return NOOK;
		}
	}
	return OK;
}


int forward_kinematics(double t[5][5],double j[7+1]) {	
	double   c1	=	C1;
	double 	c2	=	C2;
	double 	c3	=	C3;
	double 	c4	=	C4;
	double 	c5  =	C5;
	double   c6	=	C6;
	double 	c7	=	C7;
	double 	c23 =	C23;
	double 	s1  =	S1;
	double 	s2	=	S2;
	double 	s3	=	S3;
	double 	s4	=	S4;
	double 	s5	=	S5;
	double 	s6	=   S6;
	double 	s7  =	S7;
	double 	s23 =	S23;
	
	double s4c5=s4*c5;
	
	double s1s2=s1*s2;
	double c1s2=c1*s2;
	
	
	double s1s4=s1*s4;
	double s1c4=s1*c4;
	double c1s4=c1*s4;
	double c1c4=c1*c4;
	double c4c5=c4*c5;
	double c4s5=c4*s5;
	
	double s1s4c5=s1s4*c5;
	double s1s4s5=s1s4*s5;
	
	double c1s4c5=c1s4*c5;
	double c1s4s5=c1s4*s5;
	
	double c23s1c4=c23*s1c4;
	double c23s1s4=c23*s1s4;
	
	double c23c1c4=c23*c1c4;
	double c23c1s4=c23*c1s4;
	
	double s23c1c5=s23*c1*c5;

	double c23c4c5=c23*c4c5;
	double c23c4s5=c23*c4s5;
	
	double s23s4s6=s23*s4*s6;
	double s23s4c6=s23*s4*c6;
	
	double s23c1=s23*c1;
	double s23s1=s23*s1;
	
	double s23s5=s23*s5;
	double s23c5=s23*c5;
	
	double c23s5=c23*s5;
	double c23c5=c23*c5;
	
	double s23c4c5=s23*c4c5;
	double s23c4s5=s23*c4s5;
	
	if (checkConstrains(j)!=OK) {
		if (DEBUG) printf ("ERROR: Incorrect angle position.\n");
		return INCORRECT_ANGLE;
	}
	
	t[1][1]=c6*(c1*(-s23s5+c23c4c5)-s1s4c5)-s6*(c23c1s4+s1c4);
	t[1][2]=s6*(c1*(-c23c4c5+s23s5)+s1s4c5)-c6*(c23c1s4+s1c4);
	t[1][3]=s5*(c23c1c4-s1s4)+s23c1c5;
	
	
	t[1][4]=LW*(c1*(c23c4s5+s23c5)-s1s4s5)+LE*(s23c1)+LS*(c1s2);
	
	t[2][1]=c6*(s1*(c23c4c5-s23s5)+c1s4c5)-s6*(c23s1s4-c1c4);
	t[2][2]=s6*(s1*(-c23c4c5+s23s5)-c1s4c5)-c6*(c23s1s4-c1c4);
	t[2][3]=s5*(c23s1c4+c1s4)+s1*(s23c5);
	
	t[2][4]=LW*(s1*(c23c4s5+s23c5)+c1s4s5)+LE*(s23s1)+LS*(s1s2);
	
	t[3][1]=c6*(-s23c4c5-c23s5)+s23s4s6;
	t[3][2]=s6*(s23c4c5+c23s5)+s23s4c6;
	t[3][3]=c23c5-s23c4s5;
	
	t[3][4]=LB+LW*(-s23c4s5+c23c5)+LE*(c23)+LS*(c2);
	
	// Normal projection
	t[4][1]=t[4][2]=t[4][3]=0;
	t[4][4]=1;
	
	return OK;
	
}

/**
 * FIXME: Just one single solution angle is allowed to return from this function, should
 * be an array of solutions.
 * 
 ***/ 
int inverse_kinematics( double j[7+1],double t[5][5]) {	

	// t is the desired position of the end effector.
	// j is the joints angle vector.
	
	//	nx	tx	bx	px
	//	ny	ty	by	py
	//	nz	tz	bz	pz
	//	0	0	0	1
	
	double ang1_1 =0;
	double ang1_2 = 0;
	
	
	double pxx = px-LW*bx;
	double pyy = py-LW*by;
	double pzz = pz-LW*bz; 
	
	
	// THETA 1
	ang1_1 = atan2( pyy, pxx);
	
	ang1_2 = atan(pyy/pxx);
	
	// I choose the one with the small absolute value for the angle.
	if ( fabs(ang1_1) > fabs(ang1_2) ) {
		j[1] = ang1_2;
	} else {
		j[1] = ang1_1;
	}

	double c1=C1;
	double s1=S1;


	double A = c1*pxx + s1*pyy;
	double B = -pzz + LB;
	
	double A2pB2 = A*A + B*B;
	
	double ALS = A*LS;
	double ALS2 = A*A*LS*LS;
	
	double LS2=LS*LS;
	double LE2=LE*LE;
	

	
	double bls2 = 2*B*LS;
	double LE2LS2A2pB2 = LE*LE - LS*LS - A2pB2;
	
	double a = -2*B*LS-LE*LE+LS*LS+A*A+B*B;
	double b =  2*B*LS-LE*LE+LS*LS+A*A+B*B;
	
	// theta 2, 2 solutions
	
	
	
	// x>0    1,4	arctan(y/x)
	// x=0    1,4   sgn(y)*pi/2
	// x<0    2,3   arctan(y/x)+sgnY*pi
	
	
	double radical = 4*A*A*LS*LS-(+2*B*LS+LE*LE-LS*LS-(A*A+B*B))*(2*B*LS-LE*LE+LS*LS+(A*A+B*B));
	
	radical = 4*A*A*LS*LS - a*b;
	 
	if (DEBUG) printf ("a:%18.10f\n", a);
	if (DEBUG) printf ("b:%18.10f\n", b);
	if (DEBUG) printf ("A:%18.10f\n", A);
	if (DEBUG) printf ("B:%18.10f\n", B);
	if (DEBUG) printf ("4*ALS2: %18.10f\n", 4*ALS2);
	if (DEBUG) printf ("a*b: %18.10f\n", a*b);
	if (DEBUG) printf ("Radical: %18.10f\n", radical);
	
	if (fabs(radical)<0.000000001 && radical<0)
		radical =0;
		
	if (radical<0) {
		printf ("Unreachable position\n");
		return UNREACH_POSITION;

	}

	
	// I try to get the value from the second angle.
	double ang2_1 = 2 * atan2( (2*ALS   +   sqrt( radical )) , a);
	//if (DEBUG) printf ("Angle 2(1): %18.10f\n", j[2]);
	
	double ang2_2 = 2 * atan2( (2*ALS   -   sqrt( radical )) , a);
	//if (DEBUG) 	printf ("Angle 2(2): %18.10f\n", j[2]);

	// I choose the small absolute value 
	if (fabs(ang2_1)<fabs(ang2_2)) {
		j[2]=ang2_1;
	} else {
		j[2]=ang2_2;
	}
	
	if (fabs(ang2_1-ang2_2)<0.0000001 && ang2_1 >0 && ang2_2 > 0) {
		j[2]=ang2_1;
	}
	
	if (fabs(ang2_1-ang2_2)<0.0000001 && ang2_1 <0 && ang2_2 < 0 ) {
		j[2]=ang2_2;
	}
	 
	
	
	
	

	
	
	//j[2]=M_PI/2;
	
	// THETA 3
	
	double a2b2LS2LE2  = (A*A + B*B - (LS2+LE2));
	double a2b2LS2LE22 = a2b2LS2LE2*a2b2LS2LE2;	
	
	double ls2le2_4 = 4*(LS*LE)*(LS*LE);
	double radical3  = (ls2le2_4 - a2b2LS2LE22 );
	
	if (DEBUG) 	printf ("Angle 3 - Analysis\n");
	if (DEBUG) 	printf ("Radical 3 - %18.10f\n", radical3);
	

	// Average heuristics.
	
	if (fabs(radical3)<0.000000001 && radical3<0)
		radical3 = 0;
	
	
	if (radical3<0) {
		printf ("Unreachable position\n");
		return UNREACH_POSITION;
	}
		



	
	double ang3_1 = atan2(  sqrt  (  radical3 ) , (a2b2LS2LE2) );		

	
	double ang3_2  = atan2( - sqrt (  radical3 ) , (a2b2LS2LE2) );	

	if (fabs(ang3_1) < fabs(ang3_2) ) {
		j[3] = ang3_1;
	} else {
		j[3] = ang3_2;
	}
	
	if ( fabs(ang3_1)-fabs(ang3_2)<0.0000001 && ang3_1*ang3_2<0 ) {
		j[3] = ang3_1;
	}
	

	/**if (pz>LB) {
		j[3]=ang3_1;
	} else if (pz<LB) {
		j[3]=ang3_2;
	} **/
	
	ang3_1=RAD2DEG(ang3_1);
	ang3_2=RAD2DEG(ang3_2);
	
	if (DEBUG) 	printf ("Angle 3 - Analysis\n");
	if (DEBUG) 	printf ("Angle 3 - 1:%18.10f\n", ang3_1);
	if (DEBUG) 	printf ("Angle 3 - 2:%18.10f\n", ang3_2);
	


		
	
	double c2=C2;
	double c3=C3;
	double c23=C23;
	
	double s2=S2;
	double s3=S3;
	double s23=S23;
	
	double c1c23=c1*c23;
	double c1s23=c1*s23;
	double s1c23=s1*c23;
	double s1s23=s1*s23;	
	
	
	// by, bx,bz,  ny, nx, ty, tx are required.

	
	double nxp=c1c23*nx+s1c23*ny-s23*nz;
	double nyp=-s1*nx+c1*ny;
	
	double txp=c1c23*tx+s1c23*ty-s23*tz;
	double typ=-s1*tx+c1*ty;
	
	double bxp=c1c23*bx+s1c23*by-s23*bz;
	double byp=-s1*bx+c1*by;
	double bzp=c1s23*bx+s1s23*by+c23*bz;
	
	double pxp = c1c23*px+s1c23*py-s23*pz+(-c23*LS*s2+s23*(c2*LS+LB));
	double pyp = -s1*px+c1*py;
	
	j[4] = atan2 ( byp , bxp );
	
	// Fourth angle is constrained so in case we have over that value change the cuadrant
	if (HEURISTICS && fabs(RAD2DEG(j[4]))>anglescheck[4]) {
		j[4]= atan2 ( -byp, -bxp );
	}
	
	
	double c4=C4;
	double s4=S4;
	
	
	j[5] = atan2 ( c4 * bxp + s4 * byp , bzp);

		
	j[6] = atan2 ( c4*nyp-s4*nxp, c4*typ-s4*txp);


	
	ang2_1=RAD2DEG(ang2_1);
	ang2_2=RAD2DEG(ang2_2);

	
	if (DEBUG) 	printf("Angle 2 -%18.10f\n",ang2_1);
	if (DEBUG) 	printf("Angle 2 -%18.10f\n",ang2_2);
	
	if ( HEURISTICS && (fabs(j[5])<0.000000001) && ((fabs(j[6]+j[4]))<0.000000001) ) {
		j[6]=j[4]=0;
	}

	
	for(int i=1;i<7;i++) {
		j[i]=RAD2DEG(j[i]);
	}
	printf("----\n");
	
	for(int i=1;i<7;i++) {
		if (DEBUG) 	printf("Angle[%d]:%18.10f\n",i,j[i]);
	}
	
	
	return OK;
}



