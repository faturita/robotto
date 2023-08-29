/**
 *************************************************************************************
 *
 * PLEASE PUT CORRECT HEADER HERE.
 * 
 * PA 10 SUPER REVOLUTION
 * 
 * みちろん、間違ったことがある。ごめんなさい
 * 
 ************************************************************************************* 
 * 
 * This program generates output for gnuplot in order to see the links as plain vector
 * in 3D (The connection with the gnuplot program is using standard unix pies).
 * 
 * Bonus:  It will be cool to use simple OpenGL techniques to render good 
 * representations of the link and get some kind of "yet another pa10 simulator".
 * 
 * This program is a mix between C and C++ (so it is a bad program in C++)...
 * 
 ************************************************************************************* 
 ***/
 
 
 
 
 
 /**
  * Brainstorming.....
  * 
  * Using the moveArm algorithm i can generate and test trajectory algoritms
  * using sequence of angles to check how the arm is supposed to be moving 
  * from that point of view.
  * 
  * TODO:
  * - Check if gnuplot can receive the parameters from the command line instead of
  * using files (it will be extremly faster)
  * - Open gnuplot using popen.
  * - Generalize the structure of the link in order to be able to receive any
  * kind of link.
  * - Check the rotations because the algorithm only works for this configuration
  * - Generalize all that things.
  ***/ 
 
#include <iostream>
#include <fstream>
#include <cstring>
#include "cmd.h"
#include "Matrix.h"
#include "Rotation.h"
#include "ASCIIConverter.h"
#include "BinaryConverter.h"
#include "MatrixConversionForm.h"
#include "MathUtils.h"
#include "math.h" 

using namespace O_SESSAME;


// DEBUG constant
#define DEBUG	0


/**
 * All this stuff should go to a math library...
 *
 **/ 
  
/*
 * Inicializa las funciones de numeros al azar.
 */
void initRandom()
{
    time_t tim;
    time(&tim);
    srand((unsigned int)(tim));
}

/*
 * Devuelve un numero al azar entre iMin e iMax.
 */
int getProb(int iMin, int iMax)
{
    return (iMin+(int)(iMax*1.0*rand()/(RAND_MAX+1.0)));
}
/**
 * P() contante entre iMin y iMax
 */
float getNaturalMinMaxProb(float iMin, float iMax)
{
	return (iMin+(float)((iMax-iMin)*1.0*rand()/(RAND_MAX+1.0)));
}


#define 	MAXLINKSIZE		30

/**
 * FIXME: Hardcoded link size.
 ***/ 
int linkSize=7;

/**
 * Read from the array string the rotation angles for the link Chain
 * 
 ***/ 
void readAngles(DirectionCosineMatrix dcm[MAXLINKSIZE],char arr[][MAXLINKSIZE]) {
	int orientation[] = {3,2,3,2,3,2,3};
	int axis[] = {123,123,123,123,123,123,123};
	int sign[] = {-1,-1,-1,-1,-1,-1,-1};

 	for(int i=0;i<linkSize;i++) {     

		double angle = atof(arr[i]);
		
		//printf ("%18.10f\t",angle);
		
		switch (orientation[i]) {
			case 1:dcm[i].Set(sign[i]*Deg2Rad(angle),0,0,123);
			break;
			case 2:dcm[i].Set(0,sign[i]*Deg2Rad(angle),0,123);
			break;
			case 3:dcm[i].Set(0,0,sign[i]*Deg2Rad(angle),123);
			break;default:break;
		}
		
	}	//printf ("\n");

}


void readAngles(DirectionCosineMatrix angles[MAXLINKSIZE],unsigned int iteration) {
	
	static ifstream myfile ("angles.dat");
	string line;
 
  	char arr[MAXLINKSIZE][MAXLINKSIZE];		// FIXME: Hardcoded.
  	if (myfile.is_open())
  	{
  		getline (myfile,line);
  	    int d=0;  
    	if (! myfile.eof() )
    	{


      		int seeker,end;
      		seeker = 0;end = line.find("\t",0);
      		 			 			
		 	for(int i=0;i<linkSize;i++) {     
  				if (DEBUG) cout << "(" << line.substr(seeker,end-seeker+1).c_str()<<")";
  				
  				strcpy(arr[i],line.substr(seeker,end-seeker+1).c_str());

  				seeker=end;
  				end=line.find("\t",seeker+1);
			}	
 				
 			if (DEBUG) cout << angles << endl;
 			      		
 			d++;
 			
            //getline (myfile,line);
    	}
    readAngles(angles,arr);
    
    if (myfile.eof()) {
    	myfile.seekg (0, ios::beg);
    }
    
    
  }
  

  else cout << "Unable to open file"; 
}

/**
 * Read the angles from a file from the file system and load them in memory.
 * 
 * These angles represent the required rotations.
 * 
 * 
 ***/
void readAngles(DirectionCosineMatrix angles[MAXLINKSIZE]) {
	
	string line;
  	ifstream myfile ("angles.dat");
  	char arr[MAXLINKSIZE][MAXLINKSIZE];		// FIXME: Hardcoded.
  	if (myfile.is_open())
  	{
  		getline (myfile,line);
  	    int d=0;  
    	while (! myfile.eof() )
    	{


      		int seeker,end;
      		seeker = 0;end = line.find("\t",0);
      		
      		
 			 			
		 	for(int i=0;i<linkSize;i++) {     
  				if (DEBUG) cout << "(" << line.substr(seeker,end-seeker+1).c_str()<<")";
  				
  				strcpy(arr[i],line.substr(seeker,end-seeker+1).c_str());

  				seeker=end;
  				end=line.find("\t",seeker+1);
			}	
 				
 			if (DEBUG) cout << angles << endl;
 			      		
 			d++;
 			
            getline (myfile,line);
    	}
    readAngles(angles,arr);
    
    myfile.close();
  }
  

  else cout << "Unable to open file"; 
}

/**
 * Reads from the file the link structure in the same format as gnuplot.
 * When reading from the file, each vector is a fixed vector, so it has
 * actually 6 components. However for this program, the vectors are not fixed, 
 * so just 3 components are preserved.  When the output is generated again
 * each vector comes after the previous one, so the link chain is preserved.
 * 
 ***/ 
void readVector(Vector linkChain[MAXLINKSIZE],char *filename) {
	
	string line;
  	ifstream myfile (filename);
  	if (myfile.is_open())
  	{
  		getline (myfile,line);
  	    int d=0;  
    	while (! myfile.eof() )
    	{
      		int seeker,end;
      		seeker = 0;end = line.find("\t",0);
      		
      		
      		linkChain[d].initialize(3);
 
 			// The 3 first components are discarded.
			for(int i=0;i<3;i++) {     
				if (DEBUG) cout << "(" << line.substr(seeker,end-seeker+1).c_str()<<")";
				//linkChain[d](i+1)=atof(line.substr(seeker,end-seeker+1).c_str());
				seeker=end;
				end=line.find("\t",seeker+1);
			}	 
			
		 	for(int i=0;i<3;i++) {     
				if (DEBUG) cout << "(" << line.substr(seeker,end-seeker+1).c_str()<<")";
				linkChain[d](i+1)=atof(line.substr(seeker,end-seeker+1).c_str())-linkChain[d](i+1);
				seeker=end;
				end=line.find("\t",seeker+1);
			}	
 				
 			if (DEBUG) cout << linkChain[d] << endl;
 			      		
 			d++;
       

            getline (myfile,line);
    	}
    myfile.close();
  }

  else cout << "Unable to open file"; 

}

/**
 * Generate Random Links sequence.
 * 
 * The link vectors are stored in the chain array object.
 * 
 * **/
void generateRandomLinks(Vector linkChain[MAXLINKSIZE])
	{
	
	Vector link(3);
	link(1) = 0;
	link(2) = 0;
	link(3) = 0;
	
	
	cout << "Initialization Random Links Arbitrary Positions...." << endl;
		

	// Random link generator.
	for (int i=0;i<linkSize;i++) {
		
		link(1)=getProb(-5,10);
		link(2)=getProb(-5,10);
		link(3)=getProb(-5,10);
		
		linkChain[i] = link;
	
		//if (DEBUG) cout << origin(1) << "\t" << origin(2) << "\t" << origin(3) << "\t";
	
		if (DEBUG) cout << link(1) << "\t" << link(2) << "\t" << link(3) << endl;
	}
	
	
}

/**
 * Save the link structure in memory to a plain text file.  The file is formatted according to gnuplot data file system.
 * **/
 
void saveLink(Vector linkChain[MAXLINKSIZE], string filename)
{
	Vector origin(3);
	origin(1)=origin(2)=origin(3)=0;
	
	Vector aux(3);
	aux(1)=aux(2)=aux(3)=0;
	
  	ofstream myfile (filename.c_str());
  	if (myfile.is_open())
  	{
    
		// Vector and link connector.
		for (int i=0;i<linkSize;i++) {
		
			aux = linkChain[i];
			
			myfile.precision(14);
			myfile.width(18);
	
			myfile << origin(1) << "\t" << origin(2) << "\t" << origin(3) << "\t";
	
			myfile << aux(1) << "\t" << aux(2) << "\t" << (aux(3)) << endl;
		
			origin += aux;
		}    
    
		myfile.close();
  	}
  else cout << "Unable to open file";

}

/**
 * Rotation algorithm....
 * 
 * For each link (vector) (For all links)
 * Go through each rotation from the one that belongs to this link, and go down
 * until the one of the first link.
 * 
 ***/ 
void rotate(DirectionCosineMatrix dcm[MAXLINKSIZE],Vector linkChain[MAXLINKSIZE]) {
 	// Applied rotations and movements to all joints.
	for (int i=0;i<linkSize;i++) {
		//DirectionCosineMatrix dcm(Deg2Rad((i == 1)?90:0),Deg2Rad(0),Deg2Rad((i == 0)?45:0), 123);
	
		Vector aux(3);
	
		aux = linkChain[i];
		for (int j=i;j>=0;j--) {
			aux = dcm[j] * aux;
		}	
	
		linkChain[i]=aux;
	
	}
 }
 
void moveArm(Vector linkChain[MAXLINKSIZE],string filename, int iteration, long delay)
{
	DirectionCosineMatrix dcm[MAXLINKSIZE];

	
	printf ("set size square;\n");
	printf ("set xlabel \"X\";\n");
	printf ("set ylabel \"Y\";\n");
	printf ("set zlabel \"Z\";\n");
	printf ("set xrange [-1.600:1.600] noreverse nowriteback;\n");
	printf ("set yrange [-1.600:1.600] noreverse nowriteback;\n");
	printf ("set zrange [-1.600:1.600] noreverse nowriteback;\n");
	printf ("set mxtics 5.000000;\n");
	printf ("set mytics 5.000000;\n");
	printf ("set mztics 5.000000;\n");
	
	
	for(unsigned long i=0;(i<iteration) || (iteration<0);i++) {
		
		readVector(linkChain,"links.rot.dat");
		saveLink(linkChain,"links.dat");
		
		printf ("splot [-1.600:1.600] [-1.600:1.600] [-0.600:1.600] \"links.dat\" using 1:2:3:4:5:6 with vectors head filled lt rgbcolor \"blue\";\n");

		flush(cout);
		
		for(long j=0;j<delay;j++) ;
		
		readAngles(dcm,i);
		
		
		rotate(dcm,linkChain);
		
		//cout << linkChain[0];
		
		printf ("splot [-1.600:1.600] [-1.600:1.600] [-0.600:1.600] \"links.rot.dat\" using 1:2:3:4:5:6 with vectors head filled lt rgbcolor \"blue\";\n");
		flush(cout);
		saveLink(linkChain,filename);
		
		for(long j=0;j<delay;j++) ;
		
		//exit(0);
	}

	
	int i;
	scanf("%i",i);
		
	printf ("quit\n");	
} 

void moveArm(Vector linkChain[MAXLINKSIZE],DirectionCosineMatrix dcm[MAXLINKSIZE],string filename, int iteration, long delay)
{
	printf ("set size square;\n");
	printf ("set xlabel \"X\";\n");
	printf ("set ylabel \"Y\";\n");
	printf ("set zlabel \"Z\";\n");
	printf ("set xrange [-1.600:1.600] noreverse nowriteback;\n");
	printf ("set yrange [-1.600:1.600] noreverse nowriteback;\n");
	printf ("set zrange [-1.600:1.600] noreverse nowriteback;\n");
	printf ("set mxtics 5.000000;\n");
	printf ("set mytics 5.000000;\n");
	printf ("set mztics 5.000000;\n");
	
	
	for(int i=0;(i<iteration) || (iteration<0);i++) {
		
		readVector(linkChain,"links.rot.dat");
		saveLink(linkChain,"links.dat");
		
		printf ("splot [-1.600:1.600] [-1.600:1.600] [-0.600:1.600] \"links.dat\" using 1:2:3:4:5:6 with vectors head filled lt rgbcolor \"blue\";\n");

		flush(cout);
		
		for(long j=0;j<delay;j++) ;
		
		rotate(dcm,linkChain);
		printf ("splot [-1.600:1.600] [-1.600:1.600] [-0.600:1.600] \"links.rot.dat\" using 1:2:3:4:5:6 with vectors head filled lt rgbcolor \"blue\";\n");
		flush(cout);
		saveLink(linkChain,filename);
		
		for(long j=0;j<delay;j++) ;
	}

	
	int i;
	scanf("%i",i);
		
	printf ("quit\n");	
}


int main(int argc, char *argv[])
{
	initRandom();

	Matrix MOI = eye(3);

	Vector linkChain[MAXLINKSIZE];
	char arr[MAXLINKSIZE][MAXLINKSIZE];
	DirectionCosineMatrix dcm[linkSize];
	
	string filename;
	
	if ( (cmd_value(argc,argv,"-help")!=NULL) || ((cmd_value(argc,argv,"-h"))!=NULL)) {
		cout << "Parameters..." << endl;
		cout << "\t" << "-h" << "\t\t\t" << "Help" << endl;
		cout << "\t" << "-r" << "\t\t\t" << "Repeat Mode" << endl;
		cout << "\t" << "-random" << "\t\t\t" << "Random" << endl;
		cout << "\t" << "-a" << "\t\t\t" << "Angles (7) "<<endl;
		cout << "\t" << "-iterate" << "\t\t\t" << "Iterate the rotation" << endl;
		exit(-1);
	}
	
	// Entering reentering mode.
	if ((cmd_value(argc,argv,"-r"))!=NULL) {
		readVector(linkChain,"links.rot.dat");
		saveLink(linkChain,"links.dat");
		
	}
	else if ((cmd_value(argc,argv,"-random"))!=NULL) {
		filename="links.dat";
		generateRandomLinks(linkChain); 
		saveLink(linkChain,filename); 
		exit(1); 		
	}
	
		 
	filename="links.rot.dat";readVector(linkChain,"links.dat"); 
	
	long delay= 999833L;
	
	if ((cmd_value(argc,argv,"-slow"))!=NULL) {
		delay = 99998333L ;
	}
	
	
	// Simulates using a file with the angle offset.
	if ((cmd_value(argc,argv,"-simulate"))!=NULL) {
		int iteration = -1;
		moveArm(linkChain,filename,iteration,delay);
		return 0;
	}
		
	
	if ((cmd_value(argc,argv,"-a"))!=NULL) {
		// FIXME:  Just take the second position from the command line which is wrong.
		for(int i=0;i<7;i++) {
			strcpy(arr[i],argv[2+i]);
		}
		readAngles(dcm,arr);
		
	} else {
		
		readAngles(dcm);
	}
	

	
	if ((cmd_value(argc,argv,"-iterate"))!=NULL) {
		int iteration = -1;
		moveArm(linkChain,dcm,filename,iteration,delay);
		return 0;		
	}
	
	if ((cmd_value(argc,argv,"-train"))!=NULL) {
		for(long delay=0;delay<30000;delay++) {
			for(int iteration=0;iteration<7;iteration++) {
				double val=0.1;
				printf ("%18.10f\t", val);
			}
			printf ("\n");
		}
		return 0;		
	}

	rotate(dcm,linkChain);
	
	

	saveLink(linkChain,filename);
	
	printf ("Press a key to terminate the program....\n");
	
	//FILE *pf = popen("/opt/gnuplot/bin/gnuplot","w");
	
	//fputs("load \"show.gplt\";\n", pf);
	
	system("/usr/bin/gnuplot  show.gplt");
	
	//fflush(pf);
	
	//int i;
	//scanf("%d",i);
	
	
	//pclose(pf);
	
	printf ("Program terminated\n");
	
    return 0;
}


