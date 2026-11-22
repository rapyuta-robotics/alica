import argparse
import os
import sys

from . import __version__
from .check import check_docs
from .compiler import CompileError, compile_dir
from .decompiler import DecompileError, decompile_docs
from .jsonio import load_dirs, model_files, write_docs
from .source import dump_yaml


def print_findings(findings, quiet):
    errors = [f for f in findings if f.severity == "error"]
    warnings = [f for f in findings if f.severity == "warning"]
    for finding in errors + ([] if quiet else warnings):
        print(finding, file=sys.stderr)
    print(f"{len(errors)} error(s), {len(warnings)} warning(s)", file=sys.stderr)
    return errors


def cmd_check(args):
    for root in args.dirs:
        if not os.path.isdir(root):
            print(f"not a directory: {root}", file=sys.stderr)
            return 2
    findings = check_docs(load_dirs(args.dirs))
    errors = print_findings(findings, args.quiet)
    if errors or (args.strict and findings):
        return 1
    return 0


def cmd_compile(args):
    if not os.path.isdir(args.src):
        print(f"not a directory: {args.src}", file=sys.stderr)
        return 2
    try:
        docs = compile_dir(args.src)
    except CompileError as e:
        for message in e.messages:
            print(f"error: {message}", file=sys.stderr)
        return 1
    if not docs:
        print(f"error: no *.alica.yaml modules with elements in {args.src}", file=sys.stderr)
        return 1
    findings = check_docs(docs)
    errors = print_findings(findings, args.quiet)
    if errors and not args.force:
        print("nothing written; fix the errors or pass --force", file=sys.stderr)
        return 1
    produced = {d.relpath for d in docs}
    stale = [rel for rel in model_files(args.out) if rel.replace(os.sep, "/") not in produced] if os.path.isdir(args.out) else []
    write_docs(docs, args.out)
    for rel in stale:
        if args.clean:
            os.remove(os.path.join(args.out, rel))
            print(f"removed stale {rel}", file=sys.stderr)
        else:
            print(f"warning: {rel} in {args.out} was not produced by this compile (use --clean to remove)", file=sys.stderr)
    print(f"wrote {len(docs)} file(s) to {args.out}", file=sys.stderr)
    return 1 if errors else 0


def cmd_decompile(args):
    if not os.path.isdir(args.dir):
        print(f"not a directory: {args.dir}", file=sys.stderr)
        return 2
    docs = load_dirs([args.dir])
    try:
        files = decompile_docs(docs, keep_ids=not args.no_ids)
    except DecompileError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    existing = [p for p in files if os.path.exists(os.path.join(args.out, p))]
    if existing and not args.force:
        print(f"error: {len(existing)} file(s) already exist in {args.out}, e.g. {existing[0]}; pass --force to overwrite", file=sys.stderr)
        return 1
    for rel, data in files.items():
        path = os.path.join(args.out, rel)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            f.write(dump_yaml(data))
    print(f"wrote {len(files)} file(s) to {args.out}", file=sys.stderr)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(prog="alicaplan", description="Compile, check and decompile ALICA plans.")
    parser.add_argument("--version", action="version", version=f"alicaplan {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("compile", help="compile *.alica.yaml sources into engine JSON")
    p.add_argument("src", help="directory with *.alica.yaml (and *.layout.yaml) files")
    p.add_argument("-o", "--out", required=True, help="output directory for the engine files")
    p.add_argument("--clean", action="store_true", help="delete model files in OUT that this compile did not produce")
    p.add_argument("--force", action="store_true", help="write the output even if the checker finds errors")
    p.add_argument("-q", "--quiet", action="store_true", help="do not print warnings")
    p.set_defaults(func=cmd_compile)

    p = sub.add_parser("check", help="check engine JSON files for broken references and wiring")
    p.add_argument("dirs", nargs="+", help="directories searched for .pml/.beh/.pty/.plh/.tsk/.cnd/.rst files")
    p.add_argument("--strict", action="store_true", help="exit with 1 on warnings too")
    p.add_argument("-q", "--quiet", action="store_true", help="do not print warnings")
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("decompile", help="turn engine JSON files into *.alica.yaml sources")
    p.add_argument("dir", help="directory with engine JSON files")
    p.add_argument("-o", "--out", required=True, help="output directory for the sources")
    p.add_argument("--no-ids", action="store_true", help="drop ids; compiling then derives new ids from names")
    p.add_argument("--force", action="store_true", help="overwrite existing files")
    p.set_defaults(func=cmd_decompile)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
