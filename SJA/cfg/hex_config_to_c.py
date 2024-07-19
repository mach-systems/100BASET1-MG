import converter as _converter
import os
from sys import argv




converter = _converter.Converter()

folder_path = os.path.dirname(__file__)
output_file = os.path.join(folder_path, "../src/NXP_SJA1105P_configStream.c")

config_files = []

for arg in range(1, len(argv)):
	filename, file_extension = os.path.splitext(argv[arg])
	if file_extension == ".c":
		output_file = argv[arg]
	elif file_extension == ".hex":
		config_files.append(argv[arg])

converter.create_c_code(config_files, output_file)


