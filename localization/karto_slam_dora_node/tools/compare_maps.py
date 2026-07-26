#!/usr/bin/env python3
import argparse,hashlib,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("first");p.add_argument("second");a=p.parse_args()
def metrics(prefix):
    p=Path(prefix); data=(p.with_suffix(".pgm")).read_bytes(); start=data.find(b"\n255\n")+5; pixels=data[start:]
    meta=json.loads(Path(str(p)+".metadata.json").read_text())
    return {"sha256":hashlib.sha256(data).hexdigest(),"occupied":pixels.count(0),"free":pixels.count(254),"unknown":pixels.count(205),"metadata":meta}
one,two=metrics(a.first),metrics(a.second);print(json.dumps({"first":one,"second":two,"identical":one==two},indent=2));raise SystemExit(0 if one==two else 1)
