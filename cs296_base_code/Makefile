.SUFFIXES: .cpp .hpp

DEBUG_MODE = true

# Programs
SHELL 	= bash
CC     	= g++
AR 	    = ar 
LD	    = ld
RM 	    = rm
CD      = cd
CP      = cp
TAR     = tar
TOUCH   = touch
ECHO	= /bin/echo
CAT	    = cat
PRINTF	= printf
SED	    = sed
DOXYGEN = doxygen
LATEX   = pdflatex
BIBTEX  = bibtex
PERF    = perf
PY      = python
######################################
# Executable Names
EXEC = $(BINDIR)/cs296_base
EXECLIB = $(BINDIR)/cs296_exelib_08

# Project Paths
PROJECT_ROOT=$(HOME)/cs296_base_code
INSTALL_PATH = $(HOME)/Desktop/install
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/objs
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc
DATADIR= $(PROJECT_ROOT)/data
SCRIPTDIR = $(PROJECT_ROOT)/scripts
PLOTDIR = $(PROJECT_ROOT)/plots
LATEXDIR = $(PROJECT_ROOT)/doc
PATCHDIR = $(PROJECT_ROOT)/patch
LATEXFILE = report_cs296_08.tex
BIBTEXFILE = ref
MAIN = $(OBJDIR)/main.o

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -Wall 
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################


SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
NOBJS := $(filter-out $(MAIN), $(OBJS))
GPTS := $(wildcard $(SCRIPTDIR)/*.gpt)
HELPERS := $(wildcard $(SCRIPTDIR)/*helper.sh)


.PHONY: all setup exe clean doc report clean_doc clean_bin clean_obj_src redo\
	create_Box2d_dir debug_prof debug_mode_install release_mode_install release_prof\
	clean_data clean_plots reset report_bak plot_python

all: debug_mode_install setup exe doc report

redo: clean all

setup:
	@$(ECHO) -n "Creating New Directories..."
	@mkdir -p  objs
	@mkdir -p  bin
	@$(ECHO) "Done"
	
dist:
	@$(TAR) -zcf $(HOME)/cs296_g08_project.tar.gz -C ../ cs296_base_code
 

install: all
	@$(CD) $(INSTALL_PATH);\
	mkdir -p doc
	@$(CP) -R doc/html  $(INSTALL_PATH)/doc/
	@$(CP) doc/report_cs296_08.pdf  $(INSTALL_PATH)/doc/
	@$(CP) -R doc/g08_report.html  $(INSTALL_PATH)/doc/
	@$(CP) -R doc/img  $(INSTALL_PATH)/doc/img
	@$(CP) -R bin  $(INSTALL_PATH)/


$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err
	

exe: setup $(OBJS)
	@$(ECHO) -n "Creating Executable without using self made library..."
	@$(CC) -o $(EXEC) $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) "Done\n"; \
	fi;
	@$(RM) -f temp.log temp.err

create_Box2d_dir:
	@$(ECHO) "Creating Box2D Build ...  "
	@$(CD) $(EXTERNAL_ROOT)/src;\
	$(RM) -rf Box2D;\
	$(TAR) -zxf Box2D.tgz;\
	mkdir -p Box2D/build296;\
	$(CD) $(PROJECT_ROOT)

debug_mode_install: create_Box2d_dir
	@$(ECHO) "--- DEBUG MODE ---"
	@$(CD) $(EXTERNAL_ROOT)/src/Box2D/build296;\
	cmake -DCMAKE_BUILD_TYPE=Debug ../;\
	make install;\
	$(CD) $(PROJECT_ROOT)
	@$(eval DEBUG_MODE=true)
	@$(ECHO) "Done"

release_mode_install: create_Box2d_dir
	@$(ECHO) "--- RELEASE MODE ---"
	@$(CD) $(EXTERNAL_ROOT)/src/Box2D/build296;\
	cmake -DCMAKE_BUILD_TYPE=Release ../;\
	make install;\
	$(CD) $(PROJECT_ROOT)
	@$(eval CPPFLAGS+=-O3)
	@$(eval DEBUG_MODE=false)
	@$(ECHO) "Done"
	

profile: exe
	@$(ECHO) "Profiling using perf ...  " $(DEBUG_MODE)
	@if $(DEBUG_MODE); \
	then $(PERF) record -e cycles:u -fg -o g08_debug_prof.dat $(EXEC) 10000;\
	else $(PERF) record -e cycles:u -fg -o g08_release_prof.dat $(EXEC) 10000;\
	fi;

debug_prof: debug_mode_install profile

release_prof: release_mode_install profile

report_bak: $(DOCDIR)/$(LATEXFILE)
	@$(ECHO) -n "Generating Latex Report ...  "
	@$(CD) $(DOCDIR);\
	$(TOUCH) null ;\
	$(LATEX) $(LATEXFILE) > null;\
	$(BIBTEX) $(BIBTEXFILE) > null;\
	$(LATEX) $(LATEXFILE) > null;\
	$(LATEX) $(LATEXFILE) > null;\
	$(RM) -f null;\
	$(CD) ..
	@$(ECHO) "Done"

report: report_bak
	@$(ECHO) -n "Generating Html Report ...  "
	@$(CD) $(SCRIPTDIR);\
	$(PY) g08_gen_html.py;\
	$(CD) ..
	@$(ECHO) "Done"

doc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf $(DOCDIR)/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"


plot_python: $(SCRIPTDIR)/g08_gen_plots.py
	@$(ECHO) "Starting to generate data and plots using python3...  "
	@$(CD) $(SCRIPTDIR);\
	$(PY) g08_gen_csv.py;\
	$(PY) g08_gen_plots.py;\
	$(CD) ..
	@$(ECHO) "Done"


clean_obj_src:
	@$(ECHO) -n "Cleaning up objects..."
	@$(RM) -rf $(OBJDIR)
	@$(ECHO) "Done"
	@$(ECHO) -n "Cleaning up source files..."
	@$(RM) -rf $(SRCDIR)/*~
	@$(ECHO) "Done"
	@$(ECHO) -n "Cleaning up temp files..."
	@$(RM) -rf *~ $(DEPS)
	@$(ECHO) "Done"

clean_doc:
	@$(ECHO) -n "Cleaning up documentation..."
	@$(RM) -rf $(DOCDIR)/html
	@$(RM) -f $(DOCDIR)/*.aux $(DOCDIR)/*.pdf $(DOCDIR)/*.dvi $(DOCDIR)/*.log $(DOCDIR)/*.html
	@$(RM) -f $(DOCDIR)/img/*.pdf $(DOCDIR)/img/g08*
	@$(ECHO) "Done"

clean_bin:
	@$(ECHO) -n "Cleaning up executables..."
	@$(RM) -rf $(BINDIR)
	@$(ECHO) "Done"

clean_data:
	@$(ECHO) -n "Cleaning up data..."
	@$(RM) -rf $(DATADIR)
	@$(ECHO) "Done"


clean_external:
	@$(ECHO) -n "Cleaning up external..."
	@$(RM) -rf $(EXTERNAL_ROOT)/include/* $(EXTERNAL_ROOT)/lib/* $(EXTERNAL_ROOT)/src/Box2D
	@$(ECHO) "Done"

clean: clean_obj_src clean_bin clean_doc clean_data

reset: clean clean_external
