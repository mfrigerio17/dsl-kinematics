package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot

class MakefileGenerator {
    static String basePath_eigen  = System::getenv("EIGEN_ROOT")
    static String basePath_iitRBD = System::getenv("IITRBD_ROOT")

    def makefileBody(Robot robot) '''
        «val String TAB = "\t"»
        «val dir_src=""»
        REMOVE = rm -f

        DIR_BIN  = ./bin
        DIR_OBJS = $(DIR_BIN)
        DIR_DEPS = $(DIR_OBJS)
        OUT_DIRS = $(DIR_BIN) $(DIR_OBJS) $(DIR_DEPS)

        CPPFLAGS = -I«basePath_eigen» -I«basePath_iitRBD»
        CXXFLAGS = -g -Wall -O3 -march=native -mtune=native -D EIGEN_NO_DEBUG

        SRCS = $(wildcard «dir_src»*.cpp)
        OBJS = $(patsubst %.cpp,$(DIR_OBJS)/%.o,$(SRCS))

        EXE_OBJS = $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::testMain(robot)».o «Names$Files$RBD::main_benchmarkID(robot)».o «Names$Files$RBD::main_sine_task_ID(robot)».o «Names$Files$RBD::main_JSIM_test(robot)».o)
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
        «val invdyn_objs = '''«Names$Files$RBD::source(robot)».o «Names$Files$LinkInertias::source(robot)».o «Names$Files::transformsSource(robot)».o'''»
        «val jsim_objs = '''«Names$Files$RBD::inertiaMatrixHeader(robot)».o «Names$Files$LinkInertias::source(robot)».o «Names$Files::transformsSource(robot)».o'''»
        executables : $(EXES)
        $(EXES) : | $(DIR_BIN)
        $(DIR_BIN)/benchmarkID : $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::main_benchmarkID(robot)».o «Names$Files$RBD::source(robot)».o «Names$Files::transformsSource(robot)».o)
        «TAB»@echo "   * Building binary $@"
        «TAB»$(BUILD)

        $(DIR_BIN)/test : $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::testMain(robot)».o «Names$Files$RBD::source(robot)».o «Names$Files::transformsSource(robot)».o)
        «TAB»@echo "   * Building binary $@"
        «TAB»$(BUILD)

        $(DIR_BIN)/sine_task_ID : $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::main_sine_task_ID(robot)».o «Names$Files$RBD::source(robot)».o «Names$Files::transformsSource(robot)».o)
        «TAB»@echo "   * Building binary $@"
        «TAB»$(BUILD)

        $(DIR_BIN)/«Names$Files$RBD::main_JSIM_test(robot)» : $(addprefix $(DIR_OBJS)/,«Names$Files$RBD::main_JSIM_test(robot)».o «jsim_objs»)
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
}