# SPDX-License-Identifier: GPL-3.0-only
# fuzz_history.py -- Summarise the log of fuzz.py
# Copyright (c) 2026 Jakob Kastelic

# Usage: python3 fuzz_history.py WORK/fuzz.log...
#
# It prints the time span of the log, the counts of verdicts by mode, the
# reject rate of the generated parts, the number of cases in each hour
# and the list of findings with their directories.  A case is one line
# of a mode other than start, pause and min; the occt lines are second
# verdicts of earlier sm and pr cases and count apart.

import re
import sys
import time
from collections import Counter, OrderedDict

LINE = re.compile(r"^(\S+) v=(\S+) n=(\d+) s=(\d+) m=(\S+) p=(\S+) "
                  r"(NEW |DUP )?(\S+) ?(.*)$")
CASE_MODES = ("sm", "pr", "asm", "mut", "args")


def stamp(s):
    return time.mktime(time.strptime(s[:19], "%Y-%m-%dT%H:%M:%S"))


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: fuzz_history.py WORK/fuzz.log...")
    verdicts = Counter()
    occt = Counter()
    hours = OrderedDict()
    findings = []
    dups = Counter()
    rejects = Counter()
    versions = Counter()
    first = last = None
    for path in sys.argv[1:]:
        with open(path, errors="replace") as f:
            for line in f:
                m = LINE.match(line.rstrip("\n"))
                if not m:
                    continue
                when, ver, n, seed, mode, prog, tag, verdict, detail = \
                    m.groups()
                t = stamp(when)
                first = t if first is None else min(first, t)
                last = t if last is None else max(last, t)
                if mode not in CASE_MODES:
                    continue
                if tag == "NEW ":
                    findings.append((when, ver, mode, prog, verdict, seed,
                                     detail))
                elif tag == "DUP ":
                    dups[(verdict, prog)] += 1
                if verdict == "REJECT":
                    why = re.sub(r"^\S*?:\d+:\s*", "", detail)
                    rejects[(mode, re.sub(r"-?\d+(\.\d+)?", "N", why))] += 1
                key = (tag or "").strip() + verdict
                if prog == "occt":
                    occt[(mode, key)] += 1
                    continue
                verdicts[(mode, key)] += 1
                versions[ver] += 1
                hour = when[:13]
                hours[hour] = hours.get(hour, 0) + 1
    if first is None:
        print("no log lines")
        return
    span = last - first
    total = sum(verdicts.values())
    print("span %s to %s, %.2f h, %d cases, %.1f cases per minute" % (
        time.strftime("%Y-%m-%d %H:%M", time.localtime(first)),
        time.strftime("%Y-%m-%d %H:%M", time.localtime(last)),
        span / 3600, total, total / max(span / 60, 1e-9)))
    print("binaries: " + ", ".join("%s (%d cases)" % kv
                                   for kv in versions.most_common()))
    print("\nverdicts by mode:")
    for (mode, key), k in sorted(verdicts.items()):
        print("  %-5s %-14s %7d" % (mode, key, k))
    print("\nOCCT verdicts of generated parts:")
    for (mode, key), k in sorted(occt.items()):
        print("  %-5s %-14s %7d" % (mode, key, k))
    for mode, prog in (("sm", "smfold"), ("pr", "smprism")):
        made = sum(k for (m, _), k in verdicts.items() if m == mode)
        rej = verdicts.get((mode, "REJECT"), 0)
        if made:
            print("reject rate %s: %d of %d, %.1f%%" % (
                mode, rej, made, 100.0 * rej / made))
    for (mode, why), k in rejects.most_common():
        print("  %-5s %5d  %s" % (mode, k, why[:100]))
    print("\ncases per hour:")
    for hour, k in hours.items():
        print("  %s:00  %6d" % (hour, k))
    print("\nfindings: %d" % len(findings))
    for when, ver, mode, prog, verdict, seed, detail in findings:
        print("  %s %s %-9s %-10s %s mode %s seed %s" % (
            when[:19], ver, verdict, prog, detail[:200], mode, seed))
    if dups:
        print("\nrepeats of findings:")
        for (verdict, prog), k in dups.most_common():
            print("  %-9s %-10s %6d" % (verdict, prog, k))


if __name__ == "__main__":
    main()
