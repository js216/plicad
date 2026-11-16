# The tests of PliCad.  make here builds the programs (../Makefile, into
# ../build) and the programs the tests need, then
# runs run.sh: every group, or those named in GROUPS.
#
# make                       every test
# make GROUPS='parts bad'    some groups, as run.sh lists them

B := ../build
CFLAGS := -std=c99 -pedantic-errors -Werror -Wall -Wextra

run: $(B)/test_arith $(B)/budget $(B)/xresize $(B)/xkey $(B)/schconn
	cd .. && sh test/run.sh $(GROUPS)

progs:
	$(MAKE) -C .. all

# The arithmetic of expressions, bends and line hits, on the library.
$(B)/test_arith: test_arith.c | progs
	$(CC) $(CFLAGS) -g -I../src $< $(B)/libplicad.a -lm -o $@

# A runner that holds each case to its budget.
$(B)/budget: budget.c | progs
	$(CC) $(CFLAGS) $< -o $@

# A resizer of a window, from outside.
$(B)/xresize: xresize.c | progs
	$(CC) $(CFLAGS) $< -lX11 -o $@

# A sender of keys and the mouse to a window, by XTest.
$(B)/xkey: xkey.c | progs
	$(CC) $(CFLAGS) $< -lXtst -lXdamage -lX11 -o $@

# A checker of a drawing against its netlist.
$(B)/schconn: schconn.c | progs
	$(CC) $(CFLAGS) $< -lm -o $@

.PHONY: run progs
