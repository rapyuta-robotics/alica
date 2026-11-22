import json
import os
from dataclasses import dataclass, field

EXTENSIONS = {
    ".pml": "plan",
    ".beh": "behaviour",
    ".pty": "plantype",
    ".plh": "placeholder",
    ".tsk": "taskrepository",
    ".cnd": "conditionrepository",
    ".rst": "roleset",
}

INT_MAX = 2147483647


def dumps(data):
    return json.dumps(data, indent=2, sort_keys=True) + "\n"


@dataclass
class Doc:
    relpath: str
    data: dict
    root: str = ""
    errors: list = field(default_factory=list)

    @property
    def ext(self):
        return os.path.splitext(self.relpath)[1]

    @property
    def kind(self):
        return EXTENSIONS[self.ext]

    @property
    def basename(self):
        return os.path.basename(self.relpath)

    @property
    def stem(self):
        return os.path.splitext(self.basename)[0]


def model_files(root):
    found = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames.sort()
        for name in sorted(filenames):
            if os.path.splitext(name)[1] in EXTENSIONS:
                full = os.path.join(dirpath, name)
                found.append(os.path.relpath(full, root))
    return found


def load_dirs(roots):
    docs = []
    for root in roots:
        for rel in model_files(root):
            path = os.path.join(root, rel)
            with open(path) as f:
                text = f.read()
            try:
                data = json.loads(text)
            except json.JSONDecodeError as e:
                doc = Doc(rel, {}, root)
                doc.errors.append(f"invalid JSON: {e}")
                docs.append(doc)
                continue
            if not isinstance(data, dict):
                doc = Doc(rel, {}, root)
                doc.errors.append("top level must be a JSON object")
                docs.append(doc)
                continue
            docs.append(Doc(rel, data, root))
    return docs


def write_docs(docs, out_dir):
    written = []
    for doc in docs:
        path = os.path.join(out_dir, doc.relpath)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        text = dumps(doc.data)
        if os.path.exists(path):
            with open(path) as f:
                if f.read() == text:
                    written.append(doc.relpath)
                    continue
        with open(path, "w") as f:
            f.write(text)
        written.append(doc.relpath)
    return written
