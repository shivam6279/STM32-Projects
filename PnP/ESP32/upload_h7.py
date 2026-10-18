# pio upload through the H7 (Controller/tools/flash_esp.py)
from os.path import join

Import("env")

script = join(env.subst("$PROJECT_DIR"), "..", "Controller", "tools", "flash_esp.py")


def Upload(source, target, env):
	images = []
	for off, img in env.get("FLASH_EXTRA_IMAGES", []):
		images += [off, env.subst(img)]
	images += [env.subst("$ESP32_APP_OFFSET"), str(source[0])]
	cmd = ["$PYTHONEXE", script, "write-flash", "-z"] + images
	return env.Execute(" ".join(f'"{c}"' for c in cmd))


env.Replace(UPLOADCMD=Upload)
