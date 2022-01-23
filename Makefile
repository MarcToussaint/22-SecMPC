BASE = ../botop/rai
BASE2 = ../botop/src

OBJS = main.o experiment.o

DEPEND = Core Algo Gui Geo Kin Optim KOMO Franka Control

include $(BASE)/build/generic.mk
