include ../../common.mk


#-----------------------------------------------------------------------------
# Program
#-----------------------------------------------------------------------------

SMP = $(SMP_BIN_PATH)/example_rrgstar_dubins_double_integrator_airplane

SMP_OBJ_COMPILE = rrgstar_dubins_double_integrator_airplane.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o \
	$(SMP_SRC_PATH)/smp/external_libraries/inc_mu_mc/ms.o \
	$(SMP_SRC_PATH)/smp/external_libraries/inc_mu_mc/pt.o

SMP_OBJ = rrgstar_dubins_double_integrator_airplane.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o \
	$(SMP_SRC_PATH)/smp/external_libraries/inc_mu_mc/ms.o \
	$(SMP_SRC_PATH)/smp/external_libraries/inc_mu_mc/pt.o




#-----------------------------------------------------------------------------
# Make the program
#-----------------------------------------------------------------------------

all: 
	$(MAKE) targets

default: 
	$(MAKE) targets	

targets:  $(SMP) 


$(SMP): $(SMP_OBJ) 
	$(LDXX) $(SMP_OBJ_COMPILE) -o $(SMP) $(LDFLAGS) 




#-----------------------------------------------------------------------------
# Cleaning
#-----------------------------------------------------------------------------

clean:
	-find ./ -name \*.o -exec rm {} \;
	-find ./ -name \*.gch -exec rm {} \;
