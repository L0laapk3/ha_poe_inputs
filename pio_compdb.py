# https://docs.platformio.org/en/latest/integration/compile_commands.html
# C:\Users\Kris\.platformio\penv\Scripts\platformio.exe run -t compiledb


import os
Import("env")

# include toolchain paths
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# override compilation DB path
env.Replace(COMPILATIONDB_PATH="compile_commands.json")