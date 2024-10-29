import os

def find_sources(root_dir):
    sources = set()
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith('.h'):
                full_path = dirpath
                relative_path = os.path.relpath(full_path, root_dir)
                sources.add(relative_path.replace("\\", "/"))
    return sorted(sources)

if __name__ == "__main__":
    root_dir = os.getcwd()
    sources = find_sources(root_dir)
    if not sources:
        print("No Header files found.")
    else:
        for i, source in enumerate(sources):
            if i < len(sources) - 1:
                print(f"-I{source}")
            else:
                print(f"-I{source}")