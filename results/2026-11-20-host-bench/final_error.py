import csv, collections, sys
g = collections.defaultdict(list)
for r in csv.DictReader(open(sys.argv[1])): g[(r["pilot"], int(r["step"]))].append((float(r["t_s"]), float(r["sp_mm"]), float(r["x_mm"])))
print("  final error (last 1 s):", "  ".join("%s%d %+.1f" % (k[0][0], k[1], sum(x - sp for t, sp, x in v if t > 4) / max(1, sum(1 for t, _, _ in v if t > 4))) for k, v in sorted(g.items())))
