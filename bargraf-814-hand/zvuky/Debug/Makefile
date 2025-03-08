################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../zvuky.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
zvuky.o

OBJS_AS_ARGS +=  \
zvuky.o

C_DEPS +=  \
zvuky.d

C_DEPS_AS_ARGS +=  \
zvuky.d

OUTPUT_FILE_PATH +=zvuky.elf

OUTPUT_FILE_PATH_AS_ARGS +=zvuky.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
./zvuky.o: .././zvuky.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 14.1.0
	$(QUOTE)C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATtiny_DFP\1.10.348\include"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall  -mmcu=attiny814  -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 14.1.0
	$(QUOTE)C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="zvuky.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections  -mmcu=attiny814   
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "zvuky.elf" "zvuky.hex"
	"C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "zvuky.elf" "zvuky.eep" || exit 0
	"C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-objdump.exe" -h -S "zvuky.elf" > "zvuky.lss"
	"C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "zvuky.elf" "zvuky.srec"
	"C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-size.exe" "zvuky.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "zvuky.elf" "zvuky.a" "zvuky.hex" "zvuky.lss" "zvuky.eep" "zvuky.map" "zvuky.srec" "zvuky.usersignatures"
	