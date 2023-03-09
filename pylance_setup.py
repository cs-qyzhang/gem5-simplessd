from m5.objects import *
import os
import sys

if len(sys.argv) < 2:
    print(
        "Please provide the build directory of gem5, e.g. X86 if ./build/X86/gem5.opt!"
    )
    print("Exit")
    exit()

cwd = os.getcwd()
py_dir = cwd + "/build/" + sys.argv[1] + "/python/m5/objects"
if not os.path.exists(py_dir):
    print(f"{py_dir} not exist!!!")
    print("Exit")
    exit()


def yes_or_no(propmt):
    while True:
        yesno = input(propmt)
        if yesno == "Y" or yesno == "y":
            break
        elif yesno == "N" or yesno == "n":
            print("Exit")
            exit()
        else:
            print("Please input Y or N")


yes_or_no(f"Make softlink in {py_dir}? (Y/N)")

prefix_len = len(cwd) + 1
objects_import = 'if __name__ == "__main__":\n'
for key, val in sys.meta_path[0].modules.items():
    if key.startswith("m5.objects"):
        file = val[0].split("/")[-1]
        py_path = py_dir + "/" + file
        if not os.path.exists(py_path):
            os.symlink(val[0], py_path)
            print(f"{py_path} -> {val[0]}")
        objects_import += f"    from {key.split('.')[-1]} import *\n"

yes_or_no(
    "Add 'from xxx import *' in src/python/m5/objects/__init__.py? (Y/N)"
)

os.system("git stash -- src/python/m5/objects/__init__.py")
print()

f = open("src/python/m5/objects/__init__.py", "a")
f.write(f"\n{objects_import}")
f.close()

print(
    f"""It's done! Add following settings in .vscode/settings.json:

"python.analysis.extraPaths": [
    "./build/{sys.argv[1]}/python",
    "./site_scons",
],"""
)
