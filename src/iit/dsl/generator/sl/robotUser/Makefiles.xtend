package iit.dsl.generator.sl.robotUser

import iit.dsl.kinDsl.Robot

class Makefiles {

    def CMakeLists(Robot robot) '''
        # Auto-generated CMake file
        #
        #
        «val namelow = robot.name.toLowerCase»
        # Project configuration
        cmake_minimum_required(VERSION 2.8)
        project(«namelow»User)
        set(EXECUTABLE_OUTPUT_PATH ..)

        set(EIGEN_ROOT   $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")

        set(GENCODELIB iitgen«namelow»)

        set(COMMON_LIBS «namelow»common SLcommon utilities)

        # Include directories
        include_directories(${SL_ROOT}/utilities/include)
        include_directories(${SL_ROOT}/SL/include)
        include_directories(${SL_ROOT}/${ROBOT}/math)
        include_directories(${SL_ROOT}/${ROBOT}/include)
        include_directories(include)

        include_directories(${EIGEN_ROOT})

        set(TASK_SOURCES
            ${SOURCE_SL_TASK}
            ${SOURCE_TASK}
            src/initUserTasks.cpp)

        set(OPENGL_SOURCES
            ${SOURCE_SL_OPENGL}
            ${SOURCE_OPENGL}
            src/initUserGraphics.cpp)

        if (XENO)

            add_executable(xenotask ${TASK_SOURCES})
            target_link_libraries(xenotask
                ${COMMON_LIBS}
                ${XENOTASK_LIBS}
                ${GENCODELIB})

            add_executable(xenoopengl ${OPENGL_SOURCES})
            target_link_libraries(xenoopengl
                ${COMMON_LIBS}
                ${XENOOPENGL_LIBS}
                ${GENCODELIB})

        else (XENO)
            # Executables:

            add_executable(xtask ${TASK_SOURCES})
            target_link_libraries(xtask
                ${COMMON_LIBS}
                ${XTASK_LIBS}
                ${GENCODELIB})

            add_executable(xopengl ${OPENGL_SOURCES})
            target_link_libraries(xopengl
                ${COMMON_LIBS}
                ${XOPENGL_LIBS}
                ${GENCODELIB})

            add_executable(xsim
                ${SOURCE_SL_SIM}
                ${SOURCE_SIM}
                src/initUserSimulation.cpp)

            target_link_libraries(xsim
                ${COMMON_LIBS}
                ${XSIM_LIBS}
                ${GENCODELIB})
        endif(XENO)
    '''


    def Makefile(Robot robot) '''
        «val nameLower = robot.name.toLowerCase»
        «val String TAB = "\t"»
        DIR_SRCS = src
        DIR_OBJS = $(MACHTYPE)
        DIR_LIBS = $(DIR_OBJS)
        DIR_BINS = $(DIR_OBJS)
        DIR_DEPS = $(DIR_OBJS)

        DIR_SLCORE = $(SL_ROOT)/SL
        DIR_SL_LIBS = $(SL_ROOT)/lib/$(MACHTYPE)

        include $(DIR_SLCORE)/Makefile.common

        INCLUDE_PATHS = src \
        $(SL_ROOT)/include \
        $(SL_ROOT)/SLRemote/include \
        $(SL_ROOT)/«nameLower»/include \
        $(SL_ROOT)/«nameLower»/math \
        /sw/include \
        /usr/X11/include \
        /usr/local/glut/include

        CPPFLAGS += $(patsubst %,-I %,$(INCLUDE_PATHS))
        CPPFLAGS += -I/usr/local/include/eigen3/
        CXXFLAGS += -g -Wall -O3 -march=native -mtune=native -D $(MACHTYPE) -D UNIX -D EIGEN_NO_DEBUG
        LDFLAGS  += -L$(SL_ROOT)/lib/$(MACHTYPE) -L/opt/local/lib -L/sw/lib -L/usr/X11/lib

        BINARIES = xtask xopengl xsimulation benchID jsimTest

        # ------- #
        # SOURCES #
        # ------- #
        # Source files common for any architecture:

        SRCS_xtask   = initUserTasks.c sample_task.c
        SRCS_xopengl = initUserGraphics.c
        SRCS_xsimulation = initUserSimulation.c

        #
        # Architecture dependent source files:
        ifeq ($(MACHTYPE),i486xeno)
            # For XENO machine:
        else
            # Plain UNIX architecture:
        endif

        targetToObjects = $(patsubst %,$(DIR_OBJS)/%.o, $(basename $(SRCS_$(1))) )

        OBJECTS    = $(sort $(foreach TARGET,$(BINARIES),$(call targetToObjects,$(TARGET)))) # sort removes duplicates
        DEPS_FILES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_DEPS)/%.d,$(OBJECTS))

        # ------------------ #
        # Required libraries #
        # ------------------ #

        LIBS_OPENGL = glut GL GLU Xmu
        LIBS_SYS = m readline curses

        LIBS_xtask_SL = SLtask SLcommon «nameLower»_task «nameLower» utility #SLRemote
        LIBS_xopengl_SL = SLopenGL SLcommon «nameLower»_openGL «nameLower» utility
        LIBS_xsimulation_SL = SLsimulation SLcommon lwpr «nameLower»_simulation «nameLower» utility

        LIBS_xtask   = $(LIBS_xtask_SL)
        LIBS_xopengl = $(LIBS_xopengl_SL) $(LIBS_OPENGL) Xinerama X11
        LIBS_xsimulation  = $(LIBS_xsimulation_SL) nsl


        ifeq ($(MACHTYPE),i486xeno) # For XENO machine:

            LIBS_xtask_SL +=
            LIBS_xopengl_SL +=
            LIBS_xsimulation_SL +=

            LIBS_SYS += iitio canfestival canfestival_unix native rtdk analogy rtdm

        else  # Plain UNIX architecture:
            LIBS_SYS += pthread rt
        endif

        # Function that takes a target name (either a lib or a binary) and returns
        # the linker options with the required libraries.
        targetToLibs = $(patsubst %,-l%,$(LIBS_$(1)) $(LIBS_SYS))

        targetToSLLibNames = $(patsubst %,$(DIR_SL_LIBS)/lib%.a,$(LIBS_$(1)_SL))

        SL_REQUIRED_LIBS = $(sort $(foreach BIN,$(BINARIES),$(call targetToSLLibNames,$(BIN))))


        # ------- #
        # Targets #
        # ------- #
        BINARY_FILE_PATTERN = $(DIR_BINS)/%
        EXECUTABLES = $(patsubst %,$(BINARY_FILE_PATTERN),$(BINARIES))

        all : $(EXECUTABLES)

        $(EXECUTABLES) : | make_folders make_«nameLower»
        $(OBJECTS) : | make_folders

        .SECONDEXPANSION:
        $(EXECUTABLES) : $(BINARY_FILE_PATTERN) : $$(call targetToSLLibNames,%) $$(call targetToObjects,%)
        «TAB»$(CXX) $(CXXFLAGS) $(LDFLAGS) $(filter %.o,$^) $(call targetToLibs,$*)  -o $@

        .SECONDEXPANSION:
        $(OBJECTS) : $(DIR_OBJS)/%.o : $$(wildcard $(DIR_SRCS)/%.c*)
        «TAB»$(CXX) $(CPPFLAGS) -MMD -MF $(DIR_DEPS)/$*.d  $(CXXFLAGS) -c $< -o $@


        ##$(SL_REQUIRED_LIBS) : make_«nameLower»

        make_«nameLower» :
        «TAB»$(MAKE) -C $(SL_ROOT)/«nameLower» all install


        -include $(DEPS_FILES)

        # -p to avoid warning if directory exists (can be improved with checks)
        make_folders :
        «TAB»@mkdir -p $(DIR_OBJS) $(DIR_LIBS) $(DIR_BINS) $(DIR_DEPS)

        clean :
        «TAB»$(REMOVE) $(OBJECTS) $(DEPS_FILES) $(EXECUTABLES)

        .PHONY = clean all make_«nameLower» make_folders
    '''



    def imakefileUnix(Robot robot) '''
        «val nameUpperCase = robot.name.toUpperCase»
        «val nameLowerCase = robot.name.toLowerCase»
        INCLUDES = -I../src \
        -I../include \
        -I$(MY_INCLUDES) \
        -I$(LAB_INCLUDES) \
        -I$(LAB_ROOT)/«nameLowerCase»/include \
        -I$(LAB_ROOT)/«nameLowerCase»/math \
        -I/sw/include \
        -I/usr/X11/include \
        -I/usr/local/glut/include \
        -I$(LAB_ROOT)/common \
        -I/usr/local/include/eigen3/ \

        CFLAGS    = $(OPTIMIZE_CC_FLAGS) $(INCLUDES) -DEIGEN_NO_DEBUG -D$(MACHTYPE)
        SRCDIR    = ../src
        LIBDIR    = $(MY_LIBDIR)/$(MACHTYPE)
        HEADERDIR = $(MY_INCLUDES)
        LDFLAGS   = -L$(MY_LIBDIR)/$(MACHTYPE) $(LAB_LIBDIR)
        LIBRARIES =
        BINDIR    = .

        LIB_TASK   = -l«nameLowerCase»_task       -l«nameLowerCase» -lSLtask -lSLcommon -lutility  -lm
        LIB_OPENGL = -l«nameLowerCase»_openGL     -l«nameLowerCase» -lSLopenGL -lSLcommon -lutility -lm $(OPENGL_LIBRARIES) -lXinerama -lX11
        LIB_SIM    = -l«nameLowerCase»_simulation -l«nameLowerCase» -lSLsimulation -lSLcommon -lutility $(COMM_LIBRARIES) -lm -llwpr

        SRCS_X«nameUpperCase»  = \
            initUserTasks.c \
            sample_task.c
        OBJS_X«nameUpperCase»  = \
            initUserTasks.o \
            sample_task.o

        SRCS_XOPENGL  = \
            initUserGraphics.c \
             userGraphics.c

        OBJS_XOPENGL  = \
            initUserGraphics.o \
            userGraphics.o \

        SRCS_XSIM  = \
               initUserSimulation.c

        OBJS_XSIM  = \
               initUserSimulation.o

        SOURCES = $(SRCS_X«nameUpperCase») $(SRCS_XOPENGL) $(SRCS_XSIM)
        OBJECTS = $(OBJS_X«nameUpperCase») $(OBJS_XOPENGL) $(OBJS_XSIM) $(MYOBJS)

        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/x«nameLowerCase»,$(BINDIR))
        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/xmotor,$(BINDIR))
        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/xvision,$(BINDIR))

        CPPProgramListTarget( xtask, $(OBJS_X«nameUpperCase») $(CPP_OBJS_X«nameUpperCase»),$(LIB_TASK) )
        CPPProgramListTarget( xopengl, $(OBJS_XOPENGL), $(LIB_OPENGL) )
        CPPProgramListTarget( xsimulation, $(OBJS_XSIM), $(LIB_SIM) )

        NormalObjRule( $(OBJECTS) )
        NormalCPPObjRule( $(CPP_OBJECTS) )'''
}