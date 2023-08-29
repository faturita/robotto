/**
 *************************************************************************************
 * I dont know which kind of header they often put in the header of the programs.
 * 
 * PLEASE PUT CORRECT HEADER HERE.
 * 
 * PA 10 SUPER REVOLUTION
 * 
 * みちろん、間違ったことがある。ごめんなさい
 * 
 * 
 ************************************************************************************* 
 ***/
 
 
/**
 * Que necesito...
 * 
 * Chequear las ecuaciones porque me estoy yendo a la mierda con los errores numericos
 * 
 * Contadores para chequear los tiempos y analizar la performance.
 * 
 * Necesito seguir al sieguiente paso.
 * 
 * Necesito ver si se puede usar C o C++
 * 
 ***/
 
// For unknown reasons it requires iosream.h and fstream.h
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "cmd.h"
#include "pa10keisan.h"


void showHomogeneousMatrix(double t[5][5]) 
{
	for(int i=1;i<5;i++) {
		printf( "|");
		for(int j=1;j<5;j++) {
			printf ("%18.10f",t[i][j]);
		}	
		printf ("|\n");
	}
}

/**
 * Straight Forward Kinematics
 * 
 * Please provide the angles with -a
 * 
 ***/ 
int main(int argc, char *argv[])
{
	
	
	// NOTE: The matrix and the angle vector start at 1 in order to 
	// make the equations more readable and in concordance with their written version.
	double t[5][5];
	double j[8];
	
	double jback[8];
	
	
	// Input j vector (joints space);
	
	if  (cmd_value(argc,argv,"-a")!=NULL) {
		
		if (argc<8) {
			printf ("Specify all the angles please !\n");
			exit(-1);
		}
		j[1]=(atof(argv[2]));
		j[2]=(atof(argv[3]));
		j[3]=(atof(argv[4]));
		j[4]=(atof(argv[5]));
		j[5]=(atof(argv[6]));
		j[6]=(atof(argv[7]));
				
	} else {
	 	// Angles for testing.
		j[1]=(0);
		j[2]=(90);
		j[3]=(0);
		j[4]=(0);
		j[5]=(0);
		j[6]=(0);
	}
	
	for(int i=1;i<7;i++) {
		printf("Angle[%d]:%18.10f\n",i,j[i]);
	}
	
	for(int i=1;i<7;i++) {
		//j[i]=DEG2RAD(j[i]);
		j[i]=((j[i]/90)*(M_PI/2));
	}
	
	if (forward_kinematics(t,j)!=OK) {
		exit(-1);
	}
	
	showHomogeneousMatrix(t);
	
	if ( cmd_value(argc,argv,"-i")!=NULL) {
		if (inverse_kinematics(jback,t)!=OK) {
			exit(-1);
		}
	}
	

	return (0);
}

