package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot

class MakefileGenerator {
    def makefileBody(Robot robot) '''
        «val String TAB = "\t"»
        «val dir_src=""»
        REMOVE = rm -f

        DIR_BIN  = ./bin
        DIR_OBJS = $(DIR_BIN)
        DIR_DEPS = $(DIR_OBJS)
        OUT_DIRS = $(DIR_BIN) $(DIR_OBJS) $(DIR_DEPS)

        CPPFLAGS = -I$(EIGEN_ROOT) -I$(IIT_RBD_ROOT)
        CXXFLAGS = -g -Wall -O3 -march=native -mtune=native -D EIGEN_NO_DEBUG

        SRCS = $(wildcard «dir_src»*.cpp)
        OBJS = $(patsubst %.cpp,$(DIR_OBJS)/%.o,$(SRCS))

        EXE_OBJS = $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::testMain(robot)».o «Names$Files$RBD::main_benchmarkID(robot)».o «Names$Files$RBD::main_sine_task_ID(robot)».o «Names$Files$RBD::main_jsim_test(robot)».o)
        LIB_OBJS = $(filter-out $(EXE_OBJS), $(OBJS))

        EXES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_BIN)/%,$(EXE_OBJS))

        LIB = lib«robot.name.toLowerCase()».a

        all : $(OBJS)

        # ------------ #
        # OBJECT FILES #
        # ------------ #
        COMPILE = $(CXX) $(CPPFLAGS) -MMD -MF $(DIR_DEPS)/$*.d  $(CXXFLAGS) -c $< -o $@

        $(OBJS) : | $(sort $(DIR_OBJS) $(DIR_DEPS))
        $(OBJS) : $(DIR_OBJS)/%.o : «dir_src»%.cpp
        «TAB»@echo "   * Compiling $< ..."
        «TAB»$(COMPILE)

        # -------- #
        # BINARIES #
        # -------- #
        BUILD = $(CXX) $(CXXFLAGS) $^ -o $@
        «val invdyn_objs = '''«Names$Files$RBD::invDynSource(robot)».o «Names$Files$RBD::inertiaSource(robot)».o «Names$Files::transformsSource(robot)».o'''»
        «val jsim_objs = '''«Names$Files$RBD::jsimHeader(robot)».o «Names$Files$RBD::inertiaSource(robot)».o «Names$Files::transformsSource(robot)».o'''»
        executables : $(EXES)
        $(EXES) : | $(DIR_BIN)
        $(DIR_BIN)/«Names$Files$RBD::main_benchmarkID(robot)» : $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::main_benchmarkID(robot)».o «invdyn_objs»)
        «TAB»@echo "   * Building binary $@"
        «TAB»$(BUILD)

        $(DIR_BIN)/«Names$Files$RBD::testMain(robot)» : $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::testMain(robot)».o «invdyn_objs»)
        «TAB»@echo "   * Building binary $@"
        «TAB»$(BUILD)

        $(DIR_BIN)/«Names$Files$RBD::main_sine_task_ID(robot)» : $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::main_sine_task_ID(robot)».o «invdyn_objs»)
        «TAB»@echo "   * Building binary $@"
        «TAB»$(BUILD)

        $(DIR_BIN)/«Names$Files$RBD::main_jsim_test(robot)» : $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::main_jsim_test(robot)».o «jsim_objs»)
        «TAB»@echo "   * Building binary $@"
        «TAB»$(BUILD)

        # ------- #
        # LIBRARY #
        # ------- #
        lib: $(DIR_BIN)/$(LIB) | $(DIR_BIN)
        $(DIR_BIN)/$(LIB) : $(LIB_OBJS)
        «TAB»@echo "   * Building library $@ ($(notdir $^))"
        «TAB»@ar cr $@ $^

        DEPS_FILES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_DEPS)/%.d,$(OBJS))
        -include $(DEPS_FILES)

        $(sort $(OUT_DIRS)) :
        «TAB»@mkdir -p $@

        clean :
        «TAB»@$(REMOVE) $(OBJS) $(DEPS_FILES) $(DIR_BIN)/$(LIB) $(EXES)

        debug:
        «TAB»@echo $(LIB_OBJS)

        .PHONY = all lib clean debug binaries
	'''

    def public CMakeFileBody(Robot rob) '''
        «val name = rob.name»
        «val nameLow = name.toLowerCase()»
        #-------------------------------------------------------------------------------
        #
        # CMake file for the autogenerated C++ code for the robot «name»
        #
        # ADVanced Robotics department (ADVR)
        # Fondazione Istituto Italiano di Tecnologia
        #
        # Author: Marco Frigerio
        # Note: this file has been generated by the Robotics Code Generator.
        #       Do not edit unless you know what you are doing.
        #-------------------------------------------------------------------------------

        # Project configuration
        cmake_minimum_required(VERSION 2.8)
        project(gen_«nameLow»)

        set(EIGEN_ROOT   $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")
        set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install «rob.name» headers")
        set(LIB_INSTALL_ROOT    /usr/local/lib/     CACHE PATH "Where to install «rob.name» library")

        SET(CMAKE_CXX_FLAGS "-g -std=c++11 -Wall -O3 -march=native -mtune=native -D EIGEN_NO_DEBUG")

        set(LIB_NAME iitgen«nameLow»)

        set(HEADERS ./«Names$Files::mainHeader(rob)».h
                    ./«Names$Files::linkDataMapHeader(rob)».h
                    ./«Names$Files::jointDataMapHeader(rob)».h
                    ./«Names$Files::transformsHeader(rob)».h
                    ./«Names$Files::parametersHeader(rob)».h
                    ./«Names$Files::jacobiansHeader(rob)».h
                    ./«Names$Files::traitsHeader(rob)».h
                    ./«Names$Files$RBD::jsimHeader(rob)».h
                    ./«Names$Files$RBD::invDynHeader(rob)».h
                    ./«Names$Files$RBD::fwdDynHeader(rob)».h
                    ./«Names$Files$RBD::inertiaHeader(rob)».h
                    ./«Names$Files$RBD::massParametersHeader(rob)».h
                    ./«Names$Files::miscHeader(rob)».h)
        set(SOURCES ./«Names$Files::transformsHeader(rob)».cpp
                    ./«Names$Files::jacobiansHeader(rob)».cpp
                    ./«Names$Files$RBD::jsimHeader(rob)».cpp
                    ./«Names$Files$RBD::invDynSource(rob)».cpp
                    ./«Names$Files$RBD::fwdDynHeader(rob)».cpp
                    ./«Names$Files$RBD::inertiaSource(rob)».cpp
                    ./«Names$Files::miscHeader(rob)».cpp)

        # Include directories
        include_directories(${EIGEN_ROOT})

        # Add library
        add_library(${LIB_NAME} SHARED ${SOURCES})

        # Install (ie copy) header files
        install(FILES ${HEADERS} DESTINATION ${HEADER_INSTALL_ROOT}/iit/robots/«nameLow»/)

        # Install the shared library
        install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${LIB_INSTALL_ROOT})
    '''

}