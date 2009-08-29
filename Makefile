LIB=modbus.lib
DEVICE=18F242

OBJS=ascii.o diag.o frame.o modbus.o rtu.o
INCS=../framework/framework.inc ../framework/macros.inc modbus.inc private.inc

AS=gpasm
ASFLAGS=-c -p p$(DEVICE) -w 2
AR=gplib
ARFLAGS=-c

$(LIB): $(OBJS)
	$(AR) $(ARFLAGS) $(LIB) $^

$(OBJS): $(INCS)

%.o : %.asm
	$(AS) $(ASFLAGS) $<

clean:
	$(RM) *.o *.lst *.lib
