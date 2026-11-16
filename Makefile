PROGS := smfold smflat tostep measure place interfere numdiff lint param \
	family assemble extrude bool fillet mold smview smdim smsheet tomesh \
	view
LIB := util font geom expr fold brep bsolid contact shape stream integ isect \
	split merge mesh obj scene moves clash bvh coat insert asm sview

# The prototypes of the schematic programs, which keep their own build,
# are none of the programs here.
PROTO := src/sch%.c src/sch%.h src/netdiff%.c src/netdiff%.h \
	man/man1/sch%.1 man/man1/netdiff%.1 man/man5/sch%.5 \
	man/man5/netlist%.5 test/sch% test/bad_sch% test/netdiff%
SRCS := $(filter-out $(PROTO),$(wildcard src/*.c))
HDRS := $(filter-out $(PROTO),$(wildcard src/*.h))

LINTED := $(filter-out test/crlf.sm test/bad_%.st $(PROTO), \
	$(wildcard src/*.[ch] \
	test/*.sm test/*.part test/*.txt test/*.params test/*.py test/*.sh \
	test/*.expected test/*.st test/*.draw test/*.scene test/*.c src/*.sh \
	man/man1/*.1 man/man5/*.5 man/man7/*.7)) \
	lib/materials $(wildcard lib/tables/*) \
	Makefile .clang-format .clang-tidy .gitignore README.md

CC := gcc

WARN := -std=c99 -pedantic-errors -Werror -Wall -Wextra -Wpedantic \
	-Wshadow -Wundef -Wmissing-prototypes -Wstrict-prototypes \
	-Wold-style-definition -Wmissing-declarations -Wpointer-arith \
	-Wfloat-equal -Wcast-qual -Wcast-align=strict -Wconversion \
	-Wsign-conversion -Warith-conversion -Wdouble-promotion -Wformat=2 \
	-Wformat-overflow=2 -Wformat-truncation=2 -Wformat-signedness \
	-Wnull-dereference -Wwrite-strings -Wredundant-decls \
	-Wnested-externs -Wbad-function-cast -Wswitch-enum -Wswitch-default \
	-Wvla -Walloca -Wlogical-op -Wduplicated-cond -Wduplicated-branches \
	-Wjump-misses-init -Wunused-macros -Wstrict-overflow=2 \
	-Wstack-usage=65536 -Winit-self -Wmissing-include-dirs -fanalyzer \
	-Wshift-overflow=2 -Wstringop-overflow=4 -Wimplicit-fallthrough=5 \
	-Wfloat-conversion -Walloc-zero -Wcast-function-type -Wuseless-cast \
	-Wtrampolines -Wenum-conversion -Wdeclaration-after-statement

TIDYWARN := $(filter-out -fanalyzer -Wlogical-op -Wduplicated-cond \
	-Wduplicated-branches -Wjump-misses-init -Wtrampolines \
	-Wuseless-cast -Wformat-overflow=2 -Wformat-truncation=2 \
	-Wstringop-overflow=4 -Wcast-align=strict \
	-Warith-conversion -Wstack-usage=65536 -Wimplicit-fallthrough=5 \
	-Wshift-overflow=2 -Walloc-zero, $(WARN))

# The library of this tree, for the programs that look in it.
DEFS := -DPLICAD_LIB='"$(CURDIR)/lib"'

CFLAGS := $(WARN) $(DEFS) -O2 -fno-common -ffunction-sections \
	-fdata-sections -MMD -MP
LDFLAGS := -s -Wl,--as-needed -Wl,--gc-sections
SAN := -fsanitize=address,undefined -fno-sanitize-recover=all -g -O1

S := build/san

all: $(addprefix build/,$(PROGS) smdraw) edaprogs

# The programs of schematics and boards keep their own build, eda.mk, in
# build/eda; their viewers also go there sanitized, as the tests run
# them.
edaprogs:
	@$(MAKE) --no-print-directory -f eda.mk eda san

build/smdraw $(S)/smdraw: src/smdraw.sh
	@mkdir -p $(dir $@)
	cp $< $@

OBJS := $(addsuffix .o,$(LIB) $(PROGS) test_arith)

$(addprefix build/,$(OBJS)): build/%.o: src/%.c
	@mkdir -p build
	$(CC) $(CFLAGS) -c $< -o $@

build/%: build/%.o $(addprefix build/,$(addsuffix .o,$(LIB)))
	$(CC) $(LDFLAGS) $^ -lm -o $@

# The helper of the tests that resizes a window from outside.
build/xresize: test/xresize.c
	@mkdir -p build
	$(CC) $(filter-out -fanalyzer,$(WARN)) $< -lX11 -o $@

# The runner of the tests that holds each case to its budget.
build/budget: test/budget.c
	@mkdir -p build
	$(CC) $(WARN) $< -o $@

# view alone draws, by X11 and OpenGL.
build/view: build/view.o $(addprefix build/,$(addsuffix .o,$(LIB)))
	$(CC) $(LDFLAGS) $^ -lX11 -lGL -lm -o $@

# The sanitized objects skip the analyzer, which the build above runs on
# every file: with the sanitizers it would take far more memory.
$(addprefix $(S)/,$(OBJS)): $(S)/%.o: src/%.c
	@mkdir -p $(S)
	$(CC) $(filter-out -fanalyzer,$(WARN)) $(DEFS) $(SAN) -fno-common \
		-MMD -MP -c $< -o $@

$(S)/%: $(S)/%.o $(addprefix $(S)/,$(addsuffix .o,$(LIB)))
	$(CC) $(SAN) $^ -lm -o $@

$(S)/view: $(S)/view.o $(addprefix $(S)/,$(addsuffix .o,$(LIB)))
	$(CC) $(SAN) $^ -lX11 -lGL -lm -o $@

check: lint format cppcheck
	@echo "check: all passed"

lint: build/lint all
	build/lint $(LINTED)
	@for p in $(filter-out view,$(PROGS)); do \
		ldd build/$$p | awk -v p=$$p '\
		!/linux-vdso|libc\.so|libm\.so|ld-linux/ \
		{ print p ": links " $$1; bad = 1 } END { exit bad }' || \
		exit 1; done
	@readelf -d build/view | awk '/NEEDED/ && \
		!/\[lib(X11|GL|m|c)\.so\.[0-9]*\]/ \
		{ print "view: links " $$5; bad = 1 } END { exit bad }'
	@echo "ok lint, libraries"

format:
	clang-format --dry-run -Werror $(SRCS) $(HDRS)

cppcheck:
	cppcheck --enable=warning,style,performance,portability,unusedFunction \
		--inconclusive --std=c99 --force --quiet --error-exitcode=1 \
		--check-level=exhaustive $(SRCS)

tidy:
	rm -rf build/tidy build/compile_commands.json
	mkdir -p build/tidy
	bear --output build/compile_commands.json -- \
		$(MAKE) --no-print-directory tidy-objs
	run-clang-tidy -quiet -p build $(SRCS)

tidy-objs: $(patsubst src/%.c,build/tidy/%.o,$(SRCS))

build/tidy/%.o: src/%.c
	clang $(TIDYWARN) $(DEFS) -c $< -o $@

sizes: all
	@cd build && size $(PROGS) | awk 'NR > 1 { s += $$1 + $$2; \
		print $$1 + $$2, $$6 } END { print s, "total, text + data" }'

clean:
	rm -rf build

.PHONY: all edaprogs check lint format cppcheck tidy tidy-objs sizes clean
.PRECIOUS: build/%.o $(S)/%.o

-include $(wildcard build/*.d $(S)/*.d)
