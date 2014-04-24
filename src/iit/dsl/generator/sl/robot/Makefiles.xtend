package iit.dsl.generator.sl.robot

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.sl.Common

class Makefiles {

    def CMakeLists(Robot robot) '''
        «val namelow = robot.name.toLowerCase»
        # Project configuration
        cmake_minimum_required(VERSION 2.8)
        project(«namelow»)
        set(EXECUTABLE_OUTPUT_PATH ..)

        set(EIGEN_ROOT   $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")
        set(GENCODELIB iitgen«namelow»)

        # Include directories
        include_directories(${SL_ROOT}/utilities/include)
        include_directories(${SL_ROOT}/lwpr/include)
        include_directories(${SL_ROOT}/SL/include)
        include_directories(math)
        include_directories(include)

        include_directories(${EIGEN_ROOT})

        # The source files directory
        set(SOURCE_DIR ${SL_ROOT}/${ROBOT}/src)

        set(SOURCES_COMMON
            ${SOURCE_SL_COMMON}
            ${SOURCE_DIR}/robcogen_globals.cpp
            ${SOURCE_DIR}/SL_user_commands.cpp
            ${SOURCE_DIR}/SL_user_common.cpp
            ${SOURCE_DIR}/SL_dynamics.cpp
            ${SOURCE_DIR}/SL_kinematics.cpp
            )

        if (XENO)
          # Add extra dependencies for Xenomai
          include_directories(${IIT_IO_ROOT})

          # Add library
          add_library(«namelow»common STATIC ${SOURCES_COMMON})

          # Add executable
          add_executable(xenomotor
            ${SOURCE_SL_MOTOR}
            src/SL_user_motor.cpp
            src/SL_user_sensor_proc_xeno.cpp)

          target_link_libraries(xenomotor
            «namelow»common SLcommon utilities iitio
            ${XENOMOTOR_LIBS}  ${GENCODELIB})

          # Add source files lists
          set(SOURCE_TASK
            ${SOURCE_DIR}/SL_user_task_xeno.cpp
            PARENT_SCOPE)

          set(SOURCE_OPENGL
            ${SOURCE_DIR}/SL_user_openGL.cpp
            PARENT_SCOPE)

          set(SOURCE_SIM
            ${SOURCE_DIR}/SL_user_simulation.cpp
            PARENT_SCOPE)

        else (XENO)
            # Add library
            add_library(«namelow»common STATIC ${SOURCES_COMMON})

            add_executable(xmotor
                ${SOURCE_SL_MOTOR}
                src/SL_user_motor.cpp
                src/SL_user_sensor_proc_unix.cpp)

            target_link_libraries(xmotor
                «namelow»common SLcommon utilities
                ${XMOTOR_LIBS} ${GENCODELIB})

          if (ROS)
            # Add executable
            add_executable(xros
              ${SOURCE_SL_ROS}
              src/SL_user_ros.cpp)

            target_link_libraries(xros
              «namelow»common SLcommon utilities
              ${ROSPACK_LIBS}
              ${XROS_LIBS})
          endif (ROS)

          # Add source files lists
          set(SOURCE_TASK
            ${SOURCE_DIR}/SL_user_task.cpp
            PARENT_SCOPE)

          set(SOURCE_OPENGL
            ${SOURCE_DIR}/SL_user_openGL.cpp
            PARENT_SCOPE)

          set(SOURCE_SIM
            ${SOURCE_DIR}/SL_user_simulation.cpp
            PARENT_SCOPE)
        endif (XENO)
    '''    

    def imakefileUnix(Robot robot) '''
        «val nameUpperCase = robot.name.toUpperCase»
        «val nameLowerCase = robot.name.toLowerCase»
        INCLUDES = -I../src -I../include -I../math \
        -I$(MY_INCLUDES) -I/usr/X11/include \
        -I/usr/local/glut/include

        CFLAGS     = $(OPTIMIZE_CC_FLAGS) $(INCLUDES) -D$(MACHTYPE)
        SRCDIR     = ../src
        LDFLAGS    = $(LAB_LIBDIR)
        LIBDIR     = $(MYLIBDIR)/$(MACHTYPE)
        HEADERDIR  = $(MYINCLUDEPATH)
        LIBRARIES  =
        BINDIR     = .

        SRCS_COMMON  = \
        SL_user_commands.c \
        SL_user_common.c \
        SL_kinematics.c \
        SL_dynamics.c \
        SL_invDynNE.c \
        SL_invDynArt.c \
        SL_forDynComp.c \
        SL_forDynArt.c

        OBJS_COMMON  = \
        SL_user_commands.o \
        SL_user_common.o \
        SL_kinematics.o \
        SL_dynamics.o \
        SL_invDynNE.o \
        SL_invDynArt.o \
        SL_forDynComp.o \
        SL_forDynArt.o

        SRCS_X«nameUpperCase» = \
        SL_main.c \
        SL_user_common.c

        OBJS_X«nameUpperCase» = \
        SL_main.o \
        SL_user_common.o

        LIBS_X«nameUpperCase» = -lSLcommon -lutility -lX11 -lm

        SOURCES  = $(SRCS_COMMON) SL_parm_estimate.c SL_user_simulation.c SL_user_openGL.c SL_main.c SL_user_task.c SL_user_sensor_proc_unix.c SL_user_motor.c SL_user_vision.c
        OBJECTS  = $(OBJS_COMMON) SL_parm_estimate.o SL_user_simulation.o SL_user_openGL.o SL_main.o SL_user_task.o SL_user_sensor_proc_unix.o SL_user_motor.o SL_user_vision.o

        HEADERS =

        LIB_MOTOR     = -lSLmotor -lSLcommon -lutility $(COMM_LIBRARIES) -lm
        LIB_VISION    = -lSLvision -lSLcommon -llwpr -lutility $(COMM_LIBRARIES) -lm

        KeepUpToDateCopy( SL_kinematics.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_dynamics.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_forDynArt.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_forDynComp.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_invDynNE.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_invDynArt.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_parm_estimate.c, $(LAB_ROOT)/SL/src, $(SRCDIR))

        ProgramListTarget( x«nameLowerCase», $(OBJS_X«nameUpperCase»), $(LIBS_X«nameUpperCase») )
        ProgramListTarget( xmotor, $(OBJS_COMMON) SL_user_motor.o SL_user_sensor_proc_unix.o ,$(LIB_MOTOR) )
        ProgramListTarget( xvision, $(OBJS_COMMON) SL_user_vision.o, $(LIB_VISION) )

        LibraryListAddTarget( «nameLowerCase», $(OBJS_COMMON), )
        LibraryListAddTarget( «nameLowerCase»_openGL, SL_user_openGL.o ,  )
        LibraryListAddTarget( «nameLowerCase»_task, SL_user_task.o ,  )
        LibraryListAddTarget( «nameLowerCase»_simulation, SL_user_simulation.o ,  )

        NormalObjRule( $(OBJECTS) )'''



    def Makefile(Robot robot) '''
        «val nameLower = robot.name.toLowerCase»
        «val String TAB = "\t"»
        DIR_SRCS = src
        DIR_HEADERS = include
        DIR_OBJS = $(MACHTYPE)
        DIR_LIBS = $(DIR_OBJS)
        DIR_BINS = $(DIR_OBJS)
        DIR_DEPS = $(DIR_OBJS)

        DIR_INSTALL_LIBS = $(SL_ROOT)/lib/$(MACHTYPE)
        DIR_INSTALL_BINS = $(SL_ROOT)/«new Common().robotUserFolderName(robot)»/$(MACHTYPE)

        DIR_SLCORE = $(SL_ROOT)/SL
        DIR_SLCORE_SRCS = $(SL_ROOT)/SL/src

        # Where to find SL libraries and data acquisition libraries:
        DIR_SL_LIBS  = $(SL_ROOT)/lib/$(MACHTYPE)
        DIR_DAQ_LIBS = $(SL_ROOT)/lib/$(MACHTYPE)

        include $(DIR_SLCORE)/Makefile.common


        INCLUDE_PATHS = include/ \
        math/ \
        $(SL_ROOT)/include/ \
        $(SL_ROOT)/utilities/include/ \
        /usr/local/glut/include/ \
        /usr/X11/include/ \
        /usr/local/RoboLLI/include \
        $(IIT_IO_ROOT) $(FANCY_IO_ROOT)

        CPPFLAGS += $(patsubst %,-I %,$(INCLUDE_PATHS))
        CXXFLAGS += -g -Wall -O3 -march=native -mtune=native -D $(MACHTYPE) -D EIGEN_NO_DEBUG -D UNIX # -D CLMC
        LDFLAGS  += -L$(SL_ROOT)/lib/$(MACHTYPE) -L/opt/local/lib -L/sw/lib -L/usr/X11/lib -L/usr/local/RoboLLI/lib

        LIBRARIES = «nameLower» «nameLower»_openGL «nameLower»_task «nameLower»_simulation
        BINARIES  = x«nameLower» xmotor

        # ------- #
        # SOURCES #
        # ------- #
        # Source files common for any architecture:

        SL_SRCS = SL_kinematics.c \
        SL_dynamics.c \
        SL_invDynNE.c \
        SL_invDynArt.c \
        SL_forDynComp.c \
        SL_forDynArt.c

        COMMON_SRCS = SL_user_commands.c SL_user_common.c\
        $(SL_SRCS)

        SRCS_«nameLower» = $(COMMON_SRCS)
        SRCS_«nameLower»_openGL = SL_user_openGL.c
        SRCS_«nameLower»_task = SL_user_task.c
        SRCS_«nameLower»_simulation = SL_user_simulation.c

        SRCS_x«nameLower»  = SL_main.c SL_user_common.c
        SRCS_xr«nameLower» = SL_user_common.c
        SRCS_xmotor  = $(COMMON_SRCS) SL_user_motor.c
        SRCS_xrmotor = $(COMMON_SRCS) SL_user_motor.c

        #
        # Architecture dependent source files:

        ifeq ($(MACHTYPE),i486xeno)
            # For XENO machine:
            COMMON_SRCS   +=
            SRCS_«nameLower»_task += SL_user_task_xeno.c
            SRCS_x«nameLower»   +=
            SRCS_xr«nameLower»  += SL_rmain.c
            SRCS_xmotor   +=
            SRCS_xrmotor  += SL_user_sensor_proc_xeno.c
        else
            # Plain UNIX architecture:
            SRCS_«nameLower»_task +=
            SRCS_xr«nameLower»    +=
            SRCS_xmotor   += SL_user_sensor_proc_unix.c
            SRCS_xrmotor  +=
        endif


        # Prepend 'SRCS_' to the name of all the libraries and binaries
        #SRCS_GROUPS  = $(patsubst %,SRCS_%,$(LIBRARIES))
        SRCS_GROUPS += $(patsubst %,SRCS_%,$(BINARIES) $(LIBRARIES))

        # This uses each of the 'SRCS_<module>' names as a variable
        #  and gets the corresponding value (which is a list of source files)
        SOURCES = $(foreach GROUP,$(SRCS_GROUPS),$(value $(GROUP)))


        # Function that takes a target name (either a lib or a binary) and returns
        # the related object files.
        # To be called with '$(call targetToObjects,<name>)'
        # First, removes the file extension from source files, then prepends the
        #  correct directory and adds the extension '.o'.
        targetToObjects = $(patsubst %,$(DIR_OBJS)/%.o, $(basename $(SRCS_$(1))) )

        # Use 'sort' because it removes duplicates
        #OBJECTS  = $(sort $(patsubst %.c,$(DIR_OBJS)/%.o,$(filter %.c,$(SOURCES))))
        #OBJECTS += $(sort $(patsubst %.cpp,$(DIR_OBJS)/%.o,$(filter %.cpp,$(SOURCES))))
        ALL_OBJECTS = $(sort $(foreach TARGET,$(LIBRARIES) $(BINARIES),$(call targetToObjects,$(TARGET))))

        SL_OBJECTS = $(patsubst %.c,$(DIR_OBJS)/%.o,$(SL_SRCS))

        OBJECTS = $(filter-out $(SL_OBJECTS),$(ALL_OBJECTS))

        DEPS_FILES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_DEPS)/%.d,$(OBJECTS) $(SL_OBJECTS))


        # ------------------ #
        # Required libraries #
        # ------------------ #
        SYS_LIBS = m readline curses

        LIBS_x«nameLower»_SL    = SLcommon utility
        LIBS_xr«nameLower»_SL   = SLcommon utility
        LIBS_xmotor_SL  = SLmotor SLcommon utility
        LIBS_xrmotor_SL = SLmotor SLcommon utility

        # Real-robot executables might need the data I/O libraries:
        #  (but actually you control the real-robot only with a xeno machine,
        #   so these vars default to empty).
        LIBS_xr«nameLower»_DAQ   =
        LIBS_xrmotor_DAQ =

        LIBS_x«nameLower»  = $(LIBS_x«nameLower»_SL)  $(LIBS_x«nameLower»_DAQ) X11
        LIBS_xr«nameLower» = $(LIBS_x«nameLower»_SL)  $(LIBS_xr«nameLower»_DAQ) X11
        LIBS_xmotor  = $(LIBS_xmotor_SL)  $(LIBS_xmotor_DAQ)  nsl
        LIBS_xrmotor = $(LIBS_xrmotor_SL) $(LIBS_xrmotor_DAQ) nsl

        ifeq ($(MACHTYPE),i486xeno) # For XENO machine:

        else  # Plain UNIX architecture:

            LIBS_xrmotor_SL +=
            SYS_LIBS += pthread rt

        endif

        # Function that takes a target name (either a lib or a binary) and returns
        # the linker options with the required libraries.
        targetToLibs = $(patsubst %,-l%,$(LIBS_$(1)) $(SYS_LIBS))

        # These return the files names of non-system libraries, i.e. libraries that might
        #  be rebuilt by this Makefile itself (i.e. SL and DAQ libraries)
        targetToDAQLibNames = $(patsubst %,$(DIR_DAQ_LIBS)/lib%.a,$(LIBS_$(1)_DAQ))
        targetToSLLibNames  = $(patsubst %,$(DIR_SL_LIBS)/lib%.a,$(LIBS_$(1)_SL))
        targetToRebuildableLibs = $(call targetToSLLibNames,$(1)) $(call targetToDAQLibNames,$(1))


        # ------- #
        # TARGETS #
        # ------- #
        LIBRARY_FILE_PATTERN = $(DIR_LIBS)/lib%.a
        STATIC_LIBS = $(patsubst %,$(LIBRARY_FILE_PATTERN),$(LIBRARIES))
        LIBS_TO_INSTALL = $(patsubst %,$(DIR_INSTALL_LIBS)/%,$(notdir $(STATIC_LIBS)))

        BINARY_FILE_PATTERN = $(DIR_BINS)/%
        EXECUTABLES = $(patsubst %,$(BINARY_FILE_PATTERN),$(BINARIES))
        BINS_TO_INSTALL = $(patsubst %,$(DIR_INSTALL_BINS)/%,$(notdir $(EXECUTABLES)))


        all : $(EXECUTABLES) $(STATIC_LIBS)

        install : all $(LIBS_TO_INSTALL) $(BINS_TO_INSTALL)

        # These 'order-only' prerequisites *ensure* (????) that all the SL stuff gets
        # always eventually updated before building HyQ stuff.
        $(STATIC_LIBS) : | make_folders SL_install_headers
        $(OBJECTS)     : | make_folders SL_install_headers
        $(EXECUTABLES) : | make_folders SL_core

        # ------------ #
        # OBJECT FILES #
        # ------------ #
        objects : $(SL_OBJECTS) $(OBJECTS)

        # Helping "macros" for .o and .d targets
        # With the -MMD flag $(CXX) produces dependencies information as a side
        # effect of compilation (check the compiler man for details).
        # Such ouptut should also include all the required SL-core headers
        # (such that if any of those headers changes, some object here will be rebuilt)
        COMPILE = $(CXX) $(CPPFLAGS) -MMD -MF $(DIR_DEPS)/$*.d  $(CXXFLAGS) -c $< -o $@

        .SECONDEXPANSION:
        $(OBJECTS) : $(DIR_OBJS)/%.o : $$(wildcard $(DIR_SRCS)/%.c*)
        «TAB»$(COMPILE)

        $(SL_OBJECTS) : $(DIR_OBJS)/%.o : $(DIR_SLCORE_SRCS)/%.c
        «TAB»$(COMPILE)


        # ---------------------- #
        # LIBRARIES AND BINARIES #
        # ---------------------- #
        .SECONDEXPANSION:
        $(STATIC_LIBS) : $(LIBRARY_FILE_PATTERN) : $$(call targetToObjects,%)
        «TAB»@echo " * Building library $@ ..."
        «TAB»@$(REMOVE) $@
        «TAB»@ar cr $@ $^

        .SECONDEXPANSION:
        $(EXECUTABLES) : $(BINARY_FILE_PATTERN) : $$(call targetToRebuildableLibs,%) $$(call targetToObjects,%)
        «TAB»@echo " + Building binary $@ ..."
        «TAB»$(CXX) $(CXXFLAGS) $(LDFLAGS) $(filter %.o,$^) $(call targetToLibs,$*)  -o $@


        # ---------- #
        # INSTALLING #
        # ---------- #
        $(LIBS_TO_INSTALL) : $(DIR_INSTALL_LIBS)/% : $(DIR_LIBS)/%
        «TAB»@echo " * Installing library $@ ..."
        «TAB»@$(REMOVE) $@
        «TAB»@$(COPY) $< $@
        «TAB»@chmod ugo-wx $@

        $(BINS_TO_INSTALL) : $(DIR_INSTALL_BINS)/% : $(DIR_BINS)/%
        «TAB»@echo " * Installing binary $@ ..."
        «TAB»@$(REMOVE) $@
        «TAB»@$(COPY) $< $@
        «TAB»@chmod ugo-w $@


        -include $(DEPS_FILES)


        # -p to avoid warning if directory exists (can be improved with checks)
        make_folders :
        «TAB»@mkdir -p $(DIR_OBJS) $(DIR_LIBS) $(DIR_BINS) $(DIR_DEPS)


        SL_install_headers :
        «TAB»$(MAKE) -C $(DIR_SLCORE) install_headers

        SL_core :
        «TAB»$(MAKE) -C $(DIR_SLCORE) all install

        # TODO: add invocation of make in the lwpr and utility sub projects
        ######################################


        makedebug :
        «TAB»@echo $(call targetToRebuildableLibs,xrmotor)

        clean :
        «TAB»$(REMOVE) $(SL_OBJECTS) $(OBJECTS) $(DEPS_FILES) $(STATIC_LIBS) $(EXECUTABLES)

        remove_sl_srcs :
        «TAB»$(REMOVE) $(patsubst %,src/%,$(SL_SRCS))

        .PHONY = all install clean makedebug SL_core SL_install_headers make_folders remove_sl_srcs
    '''

}