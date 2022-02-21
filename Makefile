BASE = ../botop/rai
BASE2 = ../botop/src

GL = 1
OBJS = main.o experiment.o

DEPEND = Core Algo Gui Geo Kin Optim KOMO Franka Control

include $(BASE)/build/generic.mk
