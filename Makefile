PROGS := smfold smflat tostep measure place interfere numdiff param \
	family assemble extrude bool fillet mold smview smdim smsheet tomesh \
	view schdraw schnet netdiff pcbplace pcbroute pcbcheck pcbgerber \
	pcbvec gbrvec vecps schview pcbview

LIB := util font geom expr fold brep bsolid contact shape stream integ isect \
	split merge mesh obj scene moves clash bvh coat insert asm sview \
	schsrc pcbsrc pcbplot vgview vecgl

CFLAGS := -std=c99 -pedantic-errors -Werror -Wall -Wextra \
	-Wshadow -Wconversion -Wundef -Wformat=2 -Wcast-qual -Wwrite-strings \
	-Wstrict-prototypes -Wmissing-prototypes -Wold-style-definition \
	-Wvla -Wlogical-op -Wduplicated-cond \
	-fsanitize=undefined -fsanitize-trap=all \
	-fno-omit-frame-pointer -g -O2 -MMD -MP

LDLIBS := -lX11 -lGL -lm

LDFLAGS := -Wl,--as-needed

all: $(addprefix build/,$(PROGS))

build/%.o: src/%.c
	@mkdir -p build
	$(CC) $(CFLAGS) -c $< -o $@

build/libplicad.a: $(addprefix build/,$(addsuffix .o,$(LIB)))
	ar rcs $@ $^

build/%: build/%.o build/libplicad.a
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

# Static Code Analysis

SRCS := $(wildcard src/*.c)
HDRS := $(wildcard src/*.h)

check: format cppcheck analyze
	@echo "check: all passed"

analyze: $(patsubst src/%.c,build/analyze/%.o,$(SRCS))

build/analyze/%.o: src/%.c
	@mkdir -p build/analyze
	$(CC) $(CFLAGS) -fanalyzer -c $< -o $@

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
	clang -std=c99 -c $< -o $@

clean:
	rm -rf build

.PHONY: all check format cppcheck analyze tidy tidy-objs clean

-include $(wildcard build/*.d)
