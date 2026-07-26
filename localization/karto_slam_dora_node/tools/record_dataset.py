#!/usr/bin/env python3
"""Canonicalize Dora raw-event JSONL from stdin into a replayable dataset."""
import argparse,json,sys,time
p=argparse.ArgumentParser();p.add_argument("output");args=p.parse_args()
with open(args.output,"w",encoding="utf-8") as out:
    for line in sys.stdin:
        if not line.strip(): continue
        event=json.loads(line); event.setdefault("arrival",time.monotonic())
        if event.get("id") not in {"LaserScan","Odometry","SaveMap"}: continue
        if isinstance(event.get("payload"),str): event["payload"]=json.loads(event["payload"])
        out.write(json.dumps(event,sort_keys=True,separators=(",",":"))+"\n");out.flush()
