include ../../common.mk


#-----------------------------------------------------------------------------
# Program
#-----------------------------------------------------------------------------

BIN = $(BIN_PATH)/example_rrg_single_integrator_mu_calculus

OBJ_COMPILE = rrg_single_integrator_mu_calculus.o \
	$(SRC_PATH)/external_libraries/kdtree/kdtree.o \
	$(SRC_PATH)/external_libraries/inc_mu_mc/ms.o \
	$(SRC_PATH)/external_libraries/inc_mu_mc/pt.o

OBJ = rrg_single_integrator_mu_calculus.o \
	$(SRC_PATH)/external_libraries/kdtree/kdtree.o \
	$(SRC_PATH)/external_libraries/inc_mu_mc/ms.o \
	$(SRC_PATH)/external_libraries/inc_mu_mc/pt.o




#-----------------------------------------------------------------------------
# Make the program
#-----------------------------------------------------------------------------

all:
	$(MAKE) targets

default:
	$(MAKE) targets

targets:  $(BIN)


$(BIN): $(OBJ)
	$(LDXX) $(OBJ_COMPILE) -o $(BIN) $(LDFLAGS)




#-----------------------------------------------------------------------------
# Cleaning
#-----------------------------------------------------------------------------

clean:
	-find ./ -name \*.o -exec rm {} \;
	-find ./ -name \*.gch -exec rm {} \;
