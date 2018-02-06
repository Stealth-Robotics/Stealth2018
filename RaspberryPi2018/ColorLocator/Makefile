
include Makefile.inc
               
MODULE   = nttest
OBJ      = $(OBJ_DIR)/$(MODULE)
MAP      = $(OBJ_DIR)/$(MODULE).map
MKDIR_P  = mkdir -p
              
C_SRC    = $(APP_DIR)ColorLocator.cpp

CPPTEST_SRC	=	./ColorLocator.cpp

OBJ_FILES	  = 	$(OBJ_DIR)/*.o

C_OBJ    = $(patsubst $(APP_DIR)%.cpp, $(OBJ_DIR)/%.o, $(C_SRC))
CPPTEST_OBJ    = $(patsubst $(APP_DIR)%.cpp, $(OBJ_DIR)/%.o, $(CPPTEST_SRC))

#############################################################################
# The main Targets
#############################################################################

.PHONY: direct 

all : colorlocator

direct:
	$(MKDIR_P) $(OBJ_DIR)/


colorlocator:	$(CPPTEST_OBJ)
	$(CC) -o ColorLocator $(CPPTEST_OBJ) $(CFLAGS) $(INC) -lpthread -L../ntcore-master/bins/ -lntcore -lwiringPi -lstdc++

compiles:	$(C_OBJ) $(EXECUTABLE)
		ar rcs $(OBJ) $(OBJ_FILES)
clean :
	rm -rf $(OBJ_DIR)/
	$(MKDIR_P) $(OBJ_DIR)/

#############################################################################
# How to build the .elf files from the given .o-files
#############################################################################

$(EXECUTABLE) : $(C_OBJ)
	echo Here

#############################################################################
# This are generic targets how to build .o files out of the given .cpp/.s files
#############################################################################

$(OBJ_DIR)/%.o : $(APP_DIR)%.cpp   
	echo Compiling $< ...
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<
