SOURCE = main.cc controller.cpp config.cpp

MCU ?= atmega168
CPPFLAGS ?= -DNDEBUG
include amat.mk
