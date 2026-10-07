#!/usr/bin/env python3
"""Download motor data from ThrustCurve.org into Rocket-Up.

Searches the public ThrustCurve API, picks the best data file for each motor (RockSim .rse
files carry mass data, so they are preferred over RASP .eng, certification data over user
contributions) and stores it in data/motors/source/. If the `rocketup` tool is available, the
file is converted to the RocketUp motor format (.rumotor) as well.

    python tools/thrustcurve_fetch.py M2020 --manufacturer Cesaroni
    python tools/thrustcurve_fetch.py J350 K550 --convert build/bin/rocketup
    python tools/thrustcurve_fetch.py --impulse-class L --diameter 54 --list

Only the Python standard library is used.
"""

import argparse
import base64
import json
import os
import re
import subprocess
import sys
import urllib.request

API = "https://www.thrustcurve.org/api/v1"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def post(endpoint, payload):
    req = urllib.request.Request(
        f"{API}/{endpoint}.json",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "User-Agent": "rocket-up/0.1"},
    )
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.load(r)


def search(args, name=None):
    q = {"availability": "all", "maxResults": 50}
    if name:
        q["commonName" if re.fullmatch(r"[A-Oa-o]\d+", name) else "designation"] = name.upper()
    if args.manufacturer:
        q["manufacturer"] = args.manufacturer
    if args.impulse_class:
        q["impulseClass"] = args.impulse_class.upper()
    if args.diameter:
        q["diameter"] = args.diameter
    return post("search", q).get("results", [])


def rank(f):
    """Higher is better: certification data first, then manufacturer data; among equally
    trusted files RockSim (.rse) wins because it also carries the mass curve."""
    score = {"cert": 20, "mfr": 10}.get(f.get("source"), 0)
    score += 4 if f.get("format") == "RockSim" else 0
    score += min(f.get("dataPoints", 0), 200) / 100.0
    return score


def safe(s):
    return re.sub(r"[^A-Za-z0-9._-]+", "_", s).strip("_")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*", help="common names (M2020) or designations (8429M2020-P)")
    ap.add_argument("--manufacturer", help="e.g. Cesaroni, AeroTech, Estes")
    ap.add_argument("--impulse-class", help="letter, e.g. K")
    ap.add_argument("--diameter", type=float, help="motor diameter in mm")
    ap.add_argument("--list", action="store_true", help="only list the matching motors")
    ap.add_argument("--out", default=os.path.join(ROOT, "data", "motors"), help="motor folder")
    ap.add_argument("--convert", help="path of the rocketup executable used to write .rumotor files")
    args = ap.parse_args()

    motors = []
    for n in args.names or [None]:
        motors += search(args, n)
    if not motors:
        print("no motor found", file=sys.stderr)
        return 1
    for m in motors:
        print(f"{m['manufacturerAbbrev']:>6} {m['designation']:<22} {m['commonName']:<7} "
              f"{m['diameter']:>5} mm  {m['totImpulseNs']:>8.1f} Ns  {m.get('burnTimeS') or 0:>5.2f} s  "
              f"files: {m.get('dataFiles', 0)}")
    if args.list:
        return 0

    src_dir = os.path.join(args.out, "source")
    os.makedirs(src_dir, exist_ok=True)
    for m in motors:
        files = post("download", {"motorIds": [m["motorId"]], "data": "file"}).get("results", [])
        if not files:
            print(f"  {m['designation']}: no data file")
            continue
        best = max(files, key=rank)
        ext = ".rse" if best["format"] == "RockSim" else ".eng"
        name = safe(f"{m['manufacturer'].split()[0]}_{m['designation']}") + ext
        path = os.path.join(src_dir, name)
        with open(path, "wb") as fh:
            fh.write(base64.b64decode(best["data"]))
        print(f"  saved {path} ({best['format']}, source {best.get('source')}, "
              f"https://www.thrustcurve.org{best.get('infoUrl', '')})")
        if args.convert:
            out = os.path.join(args.out, os.path.splitext(name)[0] + ".rumotor")
            subprocess.run([os.path.abspath(args.convert), "motor", path, "--convert", out], check=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
