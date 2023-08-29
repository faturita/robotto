# Roboto no Koto desu
# Testing Makefile
#
# $Author: M $
# $Revision: 1.12 $
# $Date: 2003/08/24 20:59:13 $

# Which compiler to use
CC      = g++
CFLAGS 		= -g -std=c++11 -fpermissive -Wall -w $(INCLUDE)
SRC		= src

INCLUDEDIR = /home/lsym/work/matrix/src/

INCLUDE 	= -I/usr/include -I$(INCLUDEDIR)/ -I$(INCLUDEDIR)/matrix -I$(INCLUDEDIR)/rotation \
		-I$(INCLUDEDIR)/kinematics -I$(INCLUDEDIR)/dynamics -I$(INCLUDEDIR)/utils \
		-I$(INCLUDEDIR)/datahandling -I$(INCLUDEDIR)/orbit -I$(INCLUDEDIR)/environment \
		-I$(INCLUDEDIR)/attitude -I$(SRC)
LIBDIR		= $(INCLUDEDIR)/../lib/
 
LDFLAGS 	= -L$(LIBDIR) \
		-losessame_rotation -losessame_matrix \
		-losessame_attitude \
		-losessame_environment \
		-losessame_dynamics -losessame_utils \
		-losessame_datahandling -losessame_orbit -losessame_math \
		-L/usr/lib -lstdc++ 

basic:$(SRC)/cmd.cpp $(SRC)/pa10keisan.cpp 
	$(CC) $(CFLAGS) $(INCLUDE) -c $(SRC)/cmd.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $(SRC)/pa10keisan.cpp 

%.o:$(SRC)/%.cpp basic
	$(CC) $(CFLAGS) $(INCLUDE) -c $(SRC)/$*.cpp

.cpp : basic
	$(CC) $(CFLAGS) $(INCLUDE) -c $<
	$(CC) -o bin/$* pa10keisan.o cmd.o $*.o ${LDFLAGS}
.o : FORCE $<
	$(CC) -o bin/$* pa10keisan.o cmd.o $*.o ${LDFLAGS} 
all: basic
	$(CC) $(CFLAGS) $(INCLUDE) -c $(SRC)/PA10Sim.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $(SRC)/linkChain.cpp 
	$(CC) -o bin/PA10Sim pa10keisan.o cmd.o PA10Sim.o ${LDFLAGS}
	$(CC) -o bin/linkChain cmd.o linkChain.o ${LDFLAGS}
clean:
	-rm *.o
	-rm bin/PA10Sim
	-rm bin/linkChain

# /*****************************************************************************
# *	$Log: Makefile,v $
# *	Revision 1.12  2003/08/24 20:59:13  nilspace
# *	Updated.
# *	
# *	Revision 1.11  2003/05/21 13:57:08  nilspace
# *	Updated to call correct libraries.
# *	
# *	Revision 1.10  2003/05/21 13:54:46  nilspace
# *	Updated libraries to be prefixed by osessame_
# *	
# *	Revision 1.9  2003/05/20 19:24:43  nilspace
# *	Updated.
# *	
# *	Revision 1.8  2003/05/19 19:20:50  nilspace
# *	Removed all the old targets and made a single target that is generalized to build any .cpp requested by 'make myfile.cpp'. Builds with the entire Open-Sessame framework which may be inefficient.
# *	
# *	Revision 1.7  2003/05/13 18:56:49  nilspace
# *	Cleaned up the build targets a little bit.
# *	
# *	Revision 1.6  2003/05/01 15:30:18  nilspace
# *	Updated to work with AttitudeSimulatorServer
# *	
# *	Revision 1.5  2003/04/29 20:57:48  nilspace
# *	Updated to work with Propagator.
# *	
# *	Revision 1.4  2003/04/27 22:04:33  nilspace
# *	Created the namespace O_SESSAME.
# *	
# *	Revision 1.3  2003/04/23 16:30:59  nilspace
# *	Various bugfixes & uploading of all changed code for new programmers.
# *	
# *	Revision 1.2  2003/04/08 22:52:39  nilspace
# *	Updated to work with Orbit Integration and Environment tests.
# *	
# *	Revision 1.1  2003/03/25 02:26:20  nilspace
# *	initial submission of test makefile.
# *	
# *	Revision 1.2  2003/03/05 20:39:42  nilspace
# *	Now creates librotation.a in the src/rotation directory.
# *	
# *	Revision 1.1  2003/03/05 20:21:44  nilspace
# *	New Makefile for making the rotation library. Prelminary version, need to add linking into a library.
# *	
# *
# ******************************************************************************/

