import os

def find_sources(root_dir):
    sources = []
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith('.c'):
                full_path = os.path.join(dirpath, filename)
                relative_path = os.path.relpath(full_path, root_dir)
                sources.append(relative_path.replace("\\", "/"))
    return sources

if __name__ == "__main__":
    root_dir = os.getcwd()
    sources = find_sources(root_dir)
    if not sources:
        print("No C source files found.")
    else:
        for i, source in enumerate(sources):
            if i < len(sources) - 1:
                print(f"{source} \\")
            else:
                print(source)