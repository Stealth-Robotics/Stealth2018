.PHONY : all clean

DEBUGCFG = _NDEBUG
ifeq "$(DEBUG)" "1"
    DEBUGCFG = _DEBUG
endif
                
ROOT_DIR     = ../
APP_DIR      = ./
OBJ_DIR      = ./bins
APP_OBJS_DIR = $(APP_DIR)/bins
                  
INC =   -I. \
        -I$(ROOT_DIR)ntcore-master/wpiutil/include \
        -I$(ROOT_DIR)ntcore-master/include

CC       = g++
AR       = ar

CFLAGS   = -std=c++11 -Wformat=2 -Wall -Wextra -pedantic \
	   -Wno-psabi -Wno-unused-parameter -fPIC -rdynamic \
           -Wno-error=deprecated-declarations -pthread -Wignored-qualifiers -Wpedantic \
     	   -std=gnu++14 -std=gnu++11 \
         -lopencv_objdetect \
          -lopencv_features2d -lopencv_imgproc \
          -lopencv_highgui -lopencv_core -lopencv_imgproc \
          -lopencv_videoio -lpthread -lopencv_imgcodecs


###           -fprofile-arcs -ftest-coverage -pg

ARFLAGS	 = -rcs

OBJCPYFLAGS = --output-target=srec

