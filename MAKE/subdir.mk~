CPP_SRCS += \
../../../progs/demos/sixense_simple3d/razer_linux_driver.cpp

OBJS += \
./razer_linux_driver.o 

CPP_DEPS += \
./razer_linux_driver.d 

razer_linux_driver.o: ../../../progs/demos/sixense_simple3d/razer_linux_driver.cpp
	g++-4.8 -I../../../../../src/sixense_simple3d/include -I../../../../../include -O0 -Wall -Wno-unknown-pragmas -Wreorder -Wunused-variable  -c -fmessage-length=0 -m64 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<" -lYARP_OS -lYARP_init
