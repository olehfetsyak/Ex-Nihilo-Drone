# Checks the docs against the code: every command line shown, every file named and every setting
# named in the README files and code comments has to be real.
#
#     python tools/check_docs.py
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the repo folder
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import run_sim
from drone_sim.params import default_params

FLAGS = {"--set", "--list", "--tests", "--tune", "--seed", "--version"}
docs = [os.path.join(root, f) for root, _, files in os.walk(ROOT) if "__pycache__" not in root and ".venv" not in root
        for f in files if f.endswith((".md", ".py")) and f != "check_docs.py"]
problems = []

# command lines: "python run_sim.py ..." up to a comment, a backtick or a gap of two spaces
commands = set()
for path in docs:
    for line in open(path, encoding="utf-8"):
        m = re.search(r"python run_sim\.py((?: (?! )[^#`\s)]+)*)", line)
        if m:
            commands.add(m.group(1).strip())
for args in sorted(commands):
    argv = args.split()
    try:
        sets = [argv[i + 1] for i, a in enumerate(argv[:-1]) if a == "--set"]
        if "name=value" in sets:
            continue   # a placeholder, not a real command
        run_sim.parse_settings(sets)
        for a in argv:
            if a.startswith("--") and a not in FLAGS:
                raise SystemExit(f"unknown flag {a}")
        if {"--list", "--tests", "--tune", "--version"} & set(argv):
            r = subprocess.run([sys.executable, "run_sim.py"] + argv, capture_output=True, text=True, timeout=300)
            if r.returncode:
                raise SystemExit(f"exit code {r.returncode}")
    except SystemExit as e:
        problems.append(f"command 'python run_sim.py {args}': {e}")

# file names
names = set()
for path in docs:
    text = open(path, encoding="utf-8").read()
    for m in re.finditer(r"[`\s(]((?:drone_sim|experiments|tests|tools)/[\w/.]+\.(?:py|md)|[\w-]+\.(?:md|png|txt))", text):
        names.add((m.group(1), path))
for name, where in sorted(names):
    if not (os.path.exists(os.path.join(ROOT, name)) or os.path.exists(os.path.join(os.path.dirname(where), name))):
        problems.append(f"file '{name}' named in {os.path.relpath(where, ROOT)} does not exist")

# setting names in the markdown
params, checked = default_params(), 0
for path in docs:
    if path.endswith(".md"):
        text = open(path, encoding="utf-8").read()
        for word in re.findall(r"`([a-z]+(?:_[a-z0-9]+)+)`", text) + re.findall(r"--set (\w+)=", text):
            checked += 1
            if word not in params and word not in ("its_name", "name"):
                problems.append(f"setting '{word}' in {os.path.relpath(path, ROOT)} does not exist")

print(f"{len(commands)} command lines, {len({n for n, _ in names})} file names and {checked} setting names checked")
for p in problems:
    print("  PROBLEM:", p)
print("all good" if not problems else f"{len(problems)} problems")
sys.exit(1 if problems else 0)
