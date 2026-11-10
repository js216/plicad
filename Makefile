PROGS := smfold smflat tostep measure place interfere numdiff lint
LIB := util geom expr fold brep bsolid contact

LINTED := $(filter-out test/crlf.sm test/bad_%.fold,$(wildcard src/*.[ch] \
	test/*.sm test/*.txt test/*.params test/*.py test/*.sh \
	test/*.expected test/*.fold)) test/colours \
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

CFLAGS := $(WARN) -Os -fno-common -ffunction-sections -fdata-sections \
	-MMD -MP
LDFLAGS := -s -Wl,--as-needed -Wl,--gc-sections
SAN := -fsanitize=address,undefined -fno-sanitize-recover=all -g -O1

S := build/san

all: $(addprefix build/,$(PROGS))

OBJS := $(addsuffix .o,$(LIB) $(PROGS) test_arith)

$(addprefix build/,$(OBJS)): build/%.o: src/%.c
	@mkdir -p build
	$(CC) $(CFLAGS) -c $< -o $@

build/%: build/%.o $(addprefix build/,$(addsuffix .o,$(LIB)))
	$(CC) $(LDFLAGS) $^ -lm -o $@

$(addprefix $(S)/,$(OBJS)): $(S)/%.o: src/%.c
	@mkdir -p $(S)
	$(CC) $(WARN) $(SAN) -fno-common -MMD -MP -c $< -o $@

$(S)/%: $(S)/%.o $(addprefix $(S)/,$(addsuffix .o,$(LIB)))
	$(CC) $(SAN) $^ -lm -o $@

check: lint format cppcheck
	@echo "check: all passed"

lint: build/lint all
	build/lint $(LINTED)
	@for p in $(PROGS); do ldd build/$$p | awk -v p=$$p '\
		!/linux-vdso|libc\.so|libm\.so|ld-linux/ \
		{ print p ": links " $$1; bad = 1 } END { exit bad }' || \
		exit 1; done
	@echo "ok lint, libraries"

format:
	clang-format --dry-run -Werror src/*.c src/*.h

cppcheck:
	cppcheck --enable=warning,style,performance,portability,unusedFunction \
		--inconclusive --std=c99 --force --quiet --error-exitcode=1 \
		--check-level=exhaustive src

tidy:
	rm -rf build/tidy build/compile_commands.json
	mkdir -p build/tidy
	bear --output build/compile_commands.json -- \
		$(MAKE) --no-print-directory tidy-objs
	run-clang-tidy -quiet -p build $(wildcard src/*.c)

tidy-objs: $(patsubst src/%.c,build/tidy/%.o,$(wildcard src/*.c))

build/tidy/%.o: src/%.c
	clang $(TIDYWARN) -c $< -o $@

sizes: all
	@cd build && size $(PROGS) | awk 'NR > 1 { s += $$1 + $$2; \
		print $$1 + $$2, $$6 } END { print s, "total, text + data" }'

clean:
	rm -rf build

.PHONY: all check lint format cppcheck tidy tidy-objs sizes clean
.PRECIOUS: build/%.o $(S)/%.o

-include $(wildcard build/*.d $(S)/*.d)
